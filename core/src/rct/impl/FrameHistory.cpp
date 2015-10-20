/*
 * FrameHistory.cpp
 *
 *  Created on: Oct 20, 2015
 *      Author: leon
 */

#include "FrameHistory.h"

using namespace std;

namespace rct {

DynamicFrameHistory::DynamicFrameHistory(const boost::posix_time::time_duration& maxStorageTime) :
        maxStorageTime(maxStorageTime) {
}

DynamicFrameHistory::~DynamicFrameHistory() {
}

TransformStorage DynamicFrameHistory::getData(boost::posix_time::ptime time) {
    TransformStorage* p_temp_1;
    TransformStorage* p_temp_2;

    int num_nodes = findClosest(p_temp_1, p_temp_2, time);
    if (num_nodes == 0) {
        return false;
    } else if (num_nodes == 1) {
        return *p_temp_1;
    } else if (num_nodes == 2) {
        if (p_temp_1->frameParent == p_temp_2->frameParent) {

            return interpolate(*p_temp_1, *p_temp_2, time);
        } else {
            return *p_temp_1;
        }
    } else {
        assert(0);
    }

    return true;
}

bool DynamicFrameHistory::insertData(const TransformStorage& new_data) {
    list<TransformStorage>::iterator storage_it = storage.begin();

    if (storage_it != storage.end()) {
        if (storage_it->time > new_data.time + maxStorageTime) {
            return false;
        }
    }

    while (storage_it != storage.end()) {
        if (storage_it->time <= new_data.time)
            break;
        storage_it++;
    }
    storage.insert(storage_it, new_data);

    pruneList();
    return true;
}

void DynamicFrameHistory::clearList() {
    storage.clear();
}

uint32_t DynamicFrameHistory::getParent(boost::posix_time::ptime time) {
    TransformStorage* p_temp_1;
    TransformStorage* p_temp_2;

    int num_nodes = findClosest(p_temp_1, p_temp_2, time);
    if (num_nodes == 0) {
        return 0;
    }

    return p_temp_1->frameParent;
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

    ros::Time latest_time = (*storage.begin()).time;
    ros::Time earliest_time = (*(storage.rbegin())).time;

    if (target_time == latest_time) {
        one = &(*storage.begin());
        return 1;
    } else if (target_time == earliest_time) {
        one = &(*storage.rbegin());
        return 1;
    }
    // Catch cases that would require extrapolation
    else if (target_time > latest_time) {
        std::stringstream ss;
        ss << "Lookup would require extrapolation into the future.  Requested time " << target_time
                << " but the latest data is at time " << latest_time;
        throw ExtrapolationException(ss.str());
    } else if (target_time < earliest_time) {
        std::stringstream ss;
        ss << "Lookup would require extrapolation into the past.  Requested time " << target_time
                << " but the earliest data is at time " << earliest_time;
        throw ExtrapolationException(ss.str());
    }

    //At least 2 values stored
    //Find the first value less than the target value
    list<TransformStorage>::iterator storage_it = storage.begin();
    while (storage_it != storage.end()) {
        if (storage_it->time <= target_time)
            break;
        storage_it++;
    }

    //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
    one = &*(storage_it); //Older
    two = &*(--storage_it); //Newer
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
    Eigen::Quaterniond q0 = one.transform.rotation();
    Eigen::Quaterniond q1 = two.transform.rotation();
    Eigen::Quaterniond q = q0.slerp(ratio, q1);

    TransformStorage output;
    output.transform.fromPositionOrientationScale(t, q, Eigen::Vector3d::Ones());
    output.time = one.time;
    output.frameParent = one.frameParent;
    output.frameChild = one.frameChild;
    return output;
}

void DynamicFrameHistory::pruneList() {
    ros::Time latest_time = storage.begin()->time;

    while (!storage.empty() && storage.back().time + maxStorageTime < latest_time) {
        storage.pop_back();
    }
}

TransformStorage StaticFrameHistory::getData(boost::posix_time::ptime time) {
    TransformStorage out(storage);
    out.time = time;
    return out;
}

bool StaticFrameHistory::insertData(const TransformStorage& new_data) {
    storage = new_data;
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
