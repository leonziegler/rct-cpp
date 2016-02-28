/*
 * FrameHistory.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: leon
 */

#include "FrameHistory.h"
#include "../Exceptions.h"

using namespace std;

namespace rct {

const rsc::logging::LoggerPtr DynamicFrameHistory::logger = rsc::logging::Logger::getLogger(
        "rct.core.DynamicFrameHistory");

DynamicFrameHistory::DynamicFrameHistory(const boost::posix_time::time_duration& maxStorageTime) :
        maxStorageTime(maxStorageTime) {
}

DynamicFrameHistory::~DynamicFrameHistory() {
}

TransformStorage DynamicFrameHistory::getData(boost::posix_time::ptime time) {
    TransformStorage* tempStore1;
    TransformStorage* tempStore2;

    int nodes = findClosest(tempStore1, tempStore2, time);
    if (nodes == 0) {
        throw RctException("No data yet");
    } else if (nodes == 1) {
        return *tempStore1;
    } else if (nodes == 2) {
        if (tempStore1->frameParent == tempStore2->frameParent) {

            return interpolate(*tempStore1, *tempStore2, time);
        } else {
            return *tempStore1;
        }
    } else {
        assert(0);
    }
    throw RctException("No data yet");
}

bool DynamicFrameHistory::insertData(const TransformStorage& data) {
    list<TransformStorage>::iterator storageIterator = storage.begin();

    if (storageIterator != storage.end()) {
        if (storageIterator->time > data.time + maxStorageTime) {
            return false;
        }
    }

    while (storageIterator != storage.end()) {
        if (storageIterator->time <= data.time)
            break;
        storageIterator++;
    }
    storage.insert(storageIterator, data);
    //RSCTRACE(this->logger, "inserted " << data.time);

    pruneList();
    return true;
}

void DynamicFrameHistory::clearList() {
    storage.clear();
}

uint32_t DynamicFrameHistory::getParent(boost::posix_time::ptime time) {
    TransformStorage* tempStore1;
    TransformStorage* tempStore2;

    int num_nodes = findClosest(tempStore1, tempStore2, time);
    if (num_nodes == 0) {
        return 0;
    }

    return tempStore1->frameParent;
}

std::pair<boost::posix_time::ptime, uint32_t> DynamicFrameHistory::getLatestTimeAndParent() {
    if (storage.empty()) {
        return std::make_pair(boost::posix_time::microsec_clock::universal_time(), 0);
    }

    const TransformStorage& ts = storage.front();
    return std::make_pair(ts.time, ts.frameParent);
}

unsigned int DynamicFrameHistory::getListLength() {
    return storage.size();
}

boost::posix_time::ptime DynamicFrameHistory::getLatestTime() {
    if (storage.empty())
        return boost::posix_time::microsec_clock::universal_time();
    return storage.front().time;
}

boost::posix_time::ptime DynamicFrameHistory::getOldestTime() {
    if (storage.empty())
        return boost::posix_time::microsec_clock::universal_time();
    return storage.back().time;
}

inline uint8_t DynamicFrameHistory::findClosest(TransformStorage*& one, TransformStorage*& two,
        boost::posix_time::ptime target_time) {

    //No values stored
    if (storage.empty()) {
        return 0;
    }

    //If time == 0 return the latest
    if (target_time.is_not_a_date_time()) {
        one = &storage.front();
        return 1;
    }

    // One value stored
    if (++storage.begin() == storage.end()) {
        TransformStorage& ts = *storage.begin();
        if (ts.time == target_time) {
            one = &ts;
            return 1;
        } else {
            std::stringstream ss;
            ss << "Lookup would require extrapolation at time " << target_time << ", but only time "
                    << ts.time << " is in the buffer";
            throw ExtrapolationException(ss.str());
        }
    }

    boost::posix_time::ptime timeLatest = (*storage.begin()).time;
    boost::posix_time::ptime timeEarliest = (*(storage.rbegin())).time;
    //RSCTRACE(this->logger, "timeLatest " << timeLatest);
    //RSCTRACE(this->logger, "timeEarliest " << timeEarliest);

    if (target_time == timeLatest) {
        one = &(*storage.begin());
        return 1;
    } else if (target_time == timeEarliest) {
        one = &(*storage.rbegin());
        return 1;
    }
    // Catch cases that would require extrapolation
    else if (target_time > timeLatest) {
        std::stringstream ss;
        ss << "Lookup would require extrapolation into the future. Requested time " << target_time
                << " but the latest data is at time " << timeLatest;
        throw ExtrapolationException(ss.str());
    } else if (target_time < timeEarliest) {
        std::stringstream ss;
        ss << "Lookup would require extrapolation into the past. Requested time " << target_time
                << " but the earliest data is at time " << timeEarliest;
        throw ExtrapolationException(ss.str());
    }

    //At least 2 values stored
    //Find the first value less than the target value
    list<TransformStorage>::iterator storageIterator = storage.begin();
    while (storageIterator != storage.end()) {
        if (storageIterator->time <= target_time)
            break;
        storageIterator++;
    }

    //Finally the case were somewhere in the middle. Guarenteed no extrapolation :-)
    one = &*(storageIterator); //Older
    two = &*(--storageIterator); //Newer
    return 2;
}

inline TransformStorage DynamicFrameHistory::interpolate(const TransformStorage& one,
        const TransformStorage& two, boost::posix_time::ptime time) {
    // Check for zero distance case
    if (two.time == one.time) {
        return two;
    }
    //Calculate the ratio
    boost::posix_time::time_duration d0 = time - one.time;
    boost::posix_time::time_duration d1 = two.time - one.time;
    double ratio = d0.fractional_seconds() / d1.fractional_seconds();
    double s = 1.0 - ratio;

    //Interpolate translation
    Eigen::Vector3d t0 = one.transform.translation();
    Eigen::Vector3d t1 = two.transform.translation();
    Eigen::Vector3d t = s * t0 + ratio * t1;

    //Interpolate rotation
    Eigen::Quaterniond q0(one.transform.rotation());
    Eigen::Quaterniond q1(two.transform.rotation());
    Eigen::Quaterniond q = q0.slerp(ratio, q1);

    TransformStorage output;
    output.transform.fromPositionOrientationScale(t, q, Eigen::Vector3d::Ones());
    output.time = one.time;
    output.frameParent = one.frameParent;
    output.frameChild = one.frameChild;
    return output;
}

void DynamicFrameHistory::pruneList() {
    boost::posix_time::ptime timeLatest = storage.begin()->time;

    while (!storage.empty() && storage.back().time + maxStorageTime < timeLatest) {
        storage.pop_back();
    }
}

const rsc::logging::LoggerPtr StaticFrameHistory::logger = rsc::logging::Logger::getLogger(
        "rct.core.StaticFrameHistory");

TransformStorage StaticFrameHistory::getData(boost::posix_time::ptime time) {
    TransformStorage out(storage);
    out.time = time;
    return out;
}

bool StaticFrameHistory::insertData(const TransformStorage& data) {
    storage = data;
    return true;
}

void StaticFrameHistory::clearList() {
}

uint32_t StaticFrameHistory::getParent(boost::posix_time::ptime time) {
    return storage.frameParent;
}

std::pair<boost::posix_time::ptime, uint32_t> StaticFrameHistory::getLatestTimeAndParent() {
    return std::make_pair(boost::posix_time::microsec_clock::universal_time(), storage.frameParent);
}

unsigned int StaticFrameHistory::getListLength() {
    return 1;
}

boost::posix_time::ptime StaticFrameHistory::getLatestTime() {
    return boost::posix_time::microsec_clock::universal_time();
}

boost::posix_time::ptime StaticFrameHistory::getOldestTime() {
    return boost::posix_time::microsec_clock::universal_time();
}
} /* namespace rct */
