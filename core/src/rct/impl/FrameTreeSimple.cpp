/*
 * FrameTreeSimple.cpp
 *
 *  Created on: Oct 19, 2015
 *      Author: leon
 */

#include "FrameTreeSimple.h"
#include "../Exceptions.h"

#include <boost/algorithm/string.hpp>

using namespace std;

namespace rct {

const rsc::logging::LoggerPtr FrameTreeSimple::logger = rsc::logging::Logger::getLogger(
        "rct.core.FrameTreeSimple");

FrameTreeSimple::FrameTreeSimple(const boost::posix_time::time_duration& cacheTime) :
        cacheTime(cacheTime) {
    frameIDs["NO_PARENT"] = 0;
    frames.push_back(FrameHistoryPtr());
    frameIDsReverse.push_back("NO_PARENT");
}

FrameTreeSimple::~FrameTreeSimple() {
}

void FrameTreeSimple::clear() {
    boost::mutex::scoped_lock lock(frameMutex);
    if (frames.size() > 1) {
        for (std::vector<FrameHistoryPtr>::iterator framesIterator = frames.begin() + 1;
                framesIterator != frames.end(); ++framesIterator) {
            if (*framesIterator)
                (*framesIterator)->clearList();
        }
    }
}

bool FrameTreeSimple::setTransform(const Transform& transform, bool isStatic) {
    Transform stripped = transform;
    stripped.setFrameParent(
            boost::algorithm::trim_left_copy_if(stripped.getFrameParent(), boost::is_any_of("/")));
    stripped.setFrameChild(
            boost::algorithm::trim_left_copy_if(stripped.getFrameChild(), boost::is_any_of("/")));

    bool errorExists = false;
    if (stripped.getFrameChild() == stripped.getFrameParent()) {
        RSCERROR(this->logger,
                "Self transform: Ignoring transform from authority \"" << stripped.getAuthority() << "\" with parent and child frame \"" << stripped.getFrameChild() << "\" because they are the same");
        errorExists = true;
    }

    if (stripped.getFrameChild() == "") {
        RSCERROR(this->logger,
                "No child name: Ignoring transform from authority \"" << stripped.getAuthority() << "\" because child frame not set");
        errorExists = true;
    }

    if (stripped.getFrameParent() == "") {
        RSCERROR(this->logger,
                "No parent frame: Ignoring transform with child frame \"" << stripped.getFrameChild() << "\"  from authority \"" << stripped.getAuthority() << "\" because parent frame is not set");
        errorExists = true;
    }

    if (errorExists)
        return false;

    {
        boost::mutex::scoped_lock lock(frameMutex);
        uint32_t frameNum = lookupOrInsertFrameNumber(stripped.getFrameChild());
        FrameHistoryPtr frame = getFrame(frameNum);
        if (frame == NULL)
            frame = allocateFrame(frameNum, isStatic);

        TransformStorage storage;
        storage.transform = stripped.getTransform();
        storage.frameParent = lookupOrInsertFrameNumber(stripped.getFrameParent());
        storage.frameChild = frameNum;
        storage.time = stripped.getTime();
        if (frame->insertData(storage)) {
            authorities[frameNum] = stripped.getAuthority();
        } else {
            RSCWARN(this->logger,
                    "Old data: Ignoring data from the past for frame " << stripped.getFrameChild() << " at time " << stripped.getTime() <<" according to authority " << stripped.getAuthority() <<"\nPossible reasons: Data is older than than allowed by the cache time parameter, the component receives outdated transformation information, something bad is going on with the system/simulation time");
            return false;
        }
    }

    transformsChangedSignal();

    return true;
}

enum WalkEnding {
    Identity, TargetParentOfSource, SourceParentOfTarget, FullPath,
};

struct TransformCollector {
    TransformCollector() :
                    quatSourceToTop(0.0, 0.0, 0.0, 1.0),
                    vectorSourceToTop(0.0, 0.0, 0.0),
                    quatTargetToTop(0.0, 0.0, 0.0, 1.0),
                    vectorTargetToTop(0.0, 0.0, 0.0),
                    quatResult(0.0, 0.0, 0.0, 1.0),
                    vecResult(0.0, 0.0, 0.0) {
    }

    uint32_t gather(FrameHistoryPtr cache, boost::posix_time::ptime time) {
        st = cache->getData(time);
        return st.frameParent;
    }

    void accumulate(bool source) {
        if (source) {
            vectorSourceToTop = st.transform.rotation() * vectorSourceToTop
                    + st.transform.translation();
            quatSourceToTop = st.transform.rotation() * quatSourceToTop;
        } else {
            vectorTargetToTop = st.transform.rotation() * vectorTargetToTop
                    + st.transform.translation();
            quatTargetToTop = st.transform.rotation() * quatTargetToTop;
        }
    }

    void finalize(WalkEnding end, boost::posix_time::ptime _time) {
        switch (end) {
        case Identity:
            break;
        case TargetParentOfSource:
            vecResult = vectorSourceToTop;
            quatResult = quatSourceToTop;
            break;
        case SourceParentOfTarget: {
            Eigen::Quaterniond quatInverseTarget = quatTargetToTop.inverse();
            Eigen::Vector3d vectorInverseTarget = quatInverseTarget * -vectorTargetToTop;
            vecResult = vectorInverseTarget;
            quatResult = quatInverseTarget;
            break;
        }
        case FullPath: {
            Eigen::Quaterniond quatInverseTarget = quatTargetToTop.inverse();
            Eigen::Vector3d vectorInverseTarget = quatInverseTarget * -vectorTargetToTop;
            vecResult = quatInverseTarget * vectorSourceToTop + vectorInverseTarget;
            quatResult = quatInverseTarget * quatSourceToTop;
        }
            break;
        };

        time = _time;
    }

    TransformStorage st;
    boost::posix_time::ptime time;
    Eigen::Quaterniond quatSourceToTop;
    Eigen::Vector3d vectorSourceToTop;
    Eigen::Quaterniond quatTargetToTop;
    Eigen::Vector3d vectorTargetToTop;

    Eigen::Quaterniond quatResult;
    Eigen::Vector3d vecResult;
};

struct CanTransformCollector {
    uint32_t gather(FrameHistoryPtr frame, boost::posix_time::ptime time) {
        return frame->getParent(time);
    }

    void accumulate(bool source) {
    }

    void finalize(WalkEnding end, boost::posix_time::ptime time) {
    }

    TransformStorage st;
};

template<typename F>
void FrameTreeSimple::walkToTopParent(F& collector, boost::posix_time::ptime time, uint32_t targetId,
        uint32_t sourceId, std::vector<uint32_t> *frameChain) const {
    if (frameChain)
        frameChain->clear();

    // Short circuit if zero length transform to allow lookups on non existant links
    if (sourceId == targetId) {
        collector.finalize(Identity, time);
        return;
    }

    //If getting the latest get the latest common time
    if (time.is_not_a_date_time() || time == boost::posix_time::from_time_t(0)) {
        time = getLatestCommonTime(targetId, sourceId);
    }

    // Walk the tree to its root from the source frame, accumulating the transform
    uint32_t frame = sourceId;
    uint32_t topParent = frame;
    uint32_t depth = 0;

    bool extrapolationMightHaveOccurred = false;

    while (frame != 0) {
        FrameHistoryPtr cache = getFrame(frame);
        if (frameChain)
            frameChain->push_back(frame);

        if (!cache) {
            // There will be no cache for the very root of the tree
            topParent = frame;
            break;
        }

        uint32_t parent = collector.gather(cache, time);
        if (parent == 0) {
            // Just break out here... there may still be a path from source -> target
            topParent = frame;
            extrapolationMightHaveOccurred = true;
            break;
        }

        // Early out... target frame is a direct parent of the source frame
        if (frame == targetId) {
            collector.finalize(TargetParentOfSource, time);
            return;
        }

        collector.accumulate(true);

        topParent = frame;
        frame = parent;

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {
            std::stringstream ss;
            ss << "The rct tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            throw LookupException(ss.str());

        }
    }

    // Now walk to the top parent from the target frame, accumulating its transform
    frame = targetId;
    depth = 0;
    std::vector<uint32_t> frameChainReverse;

    while (frame != topParent) {
        FrameHistoryPtr cache = getFrame(frame);
        if (frameChain)
            frameChainReverse.push_back(frame);

        if (!cache) {
            break;
        }

        uint32_t parent = collector.gather(cache, time);
        if (parent == 0) {

            std::stringstream ss;
            ss << "Extrapolation error when looking up transform from frame ["
                    << lookupFrameString(sourceId) << "] to frame ["
                    << lookupFrameString(targetId) << "]";
            throw ExtrapolationException(ss.str());
        }

        // Early out... source frame is a direct parent of the target frame
        if (frame == sourceId) {
            collector.finalize(SourceParentOfTarget, time);
            if (frameChain) {
                frameChain->swap(frameChainReverse);
            }
            return;
        }

        collector.accumulate(false);

        frame = parent;

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {

            std::stringstream ss;
            ss << "The rct tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            throw LookupException(ss.str());
        }
    }

    if (frame != topParent) {
        if (extrapolationMightHaveOccurred) {
            std::stringstream ss;
            ss << "Extrapolation error when looking up transform from frame ["
                    << lookupFrameString(sourceId) << "] to frame ["
                    << lookupFrameString(targetId) << "]";
            throw ExtrapolationException(ss.str());

        }

        string err(
                "Could not find a connection between '" + lookupFrameString(targetId) + "' and '"
                        + lookupFrameString(sourceId)
                        + "' because they are not part of the same tree."
                        + "Rct has two or more unconnected trees.");
        throw ConnectivityException(err);
    }

    collector.finalize(FullPath, time);
    if (frameChain) {
        // Pruning: Compare the chains starting at the parent (end) until they differ
        int m = frameChainReverse.size() - 1;
        int n = frameChain->size() - 1;
        for (; m >= 0 && n >= 0; --m, --n) {
            if ((*frameChain)[n] != frameChainReverse[m])
                break;
        }
        // Erase all duplicate items from frame_chain
        if (n > 0)
            frameChain->erase(frameChain->begin() + (n - 1), frameChain->end());

        if (m < frameChainReverse.size()) {
            for (int i = m; i >= 0; --i) {
                frameChain->push_back(frameChainReverse[i]);
            }
        }
    }
}

Transform FrameTreeSimple::lookupTransform(const std::string& targetFrame,
        const std::string& sourceFrame, const boost::posix_time::ptime& time) const {
    boost::mutex::scoped_lock lock(frameMutex);

    RSCTRACE(this->logger, "lookup " << targetFrame << " -> " << sourceFrame)

    if (targetFrame == sourceFrame) {
        Eigen::Affine3d transIdent;
        transIdent.setIdentity();
        Transform identity(transIdent, targetFrame, sourceFrame, time);

        if (time.is_not_a_date_time() || time == boost::posix_time::from_time_t(0)) {
            uint32_t target_id = lookupFrameNumber(targetFrame);
            FrameHistoryPtr cache = getFrame(target_id);
            if (cache)
                identity.setTime(cache->getLatestTime());
        }

        return identity;
    }

    //Identify case does not need to be validated above
    uint32_t targetId = validateFrameId("lookupTransform argument target_frame", targetFrame);
    uint32_t sourceId = validateFrameId("lookupTransform argument source_frame", sourceFrame);

    TransformCollector collector;
    walkToTopParent(collector, time, targetId, sourceId, NULL);

    Eigen::Affine3d trans;
    trans.fromPositionOrientationScale(collector.vecResult, collector.quatResult,
            Eigen::Vector3d::Ones());
    Transform transformOut(trans, targetFrame, sourceFrame, collector.time);
    return transformOut;
}

Transform FrameTreeSimple::lookupTransform(const std::string& targetFrame,
        const boost::posix_time::ptime& targetTime, const std::string& sourceFrame,
        const boost::posix_time::ptime& sourceTime, const std::string& fixedFrame) const {
    RSCTRACE(this->logger, "lookup " << targetFrame << " -> " << sourceFrame)

    validateFrameId("lookupTransform argument targetFrame", targetFrame);
    validateFrameId("lookupTransform argument sourceFrame", sourceFrame);
    validateFrameId("lookupTransform argument fixedFrame", fixedFrame);

    Transform output;
    Transform temp1 = lookupTransform(fixedFrame, sourceFrame, sourceTime);
    Transform temp2 = lookupTransform(targetFrame, fixedFrame, targetTime);

    output.setTransform(temp2.getTransform() * temp1.getTransform());
    output.setTime(temp2.getTime());
    output.setFrameParent(targetFrame);
    output.setFrameChild(sourceFrame);
    return output;
}

bool FrameTreeSimple::canTransformNoLock(uint32_t targetId, uint32_t sourceId,
        const boost::posix_time::ptime& time) const {
    if (targetId == 0 || sourceId == 0) {
        return false;
    }

    if (targetId == sourceId) {
        return true;
    }

    CanTransformCollector collector;
    try {
        walkToTopParent(collector, time, targetId, sourceId, NULL);
    } catch (RctException &e) {
        RSCDEBUG(logger, "Error while checking if transform is available: " << e.what());
        return false;
    }

    return true;
}

bool FrameTreeSimple::canTransformInternal(uint32_t targetId, uint32_t sourceId,
        const boost::posix_time::ptime& time) const {
    boost::mutex::scoped_lock lock(frameMutex);
    return canTransformNoLock(targetId, sourceId, time);
}

bool FrameTreeSimple::canTransform(const std::string& targetFrame, const std::string& sourceFrame,
        const boost::posix_time::ptime& time) const {
    // Short circuit if target_frame == source_frame
    if (targetFrame == sourceFrame)
        return true;

    if (warnFrameId("canTransform argument targetFrame", targetFrame))
        return false;
    if (warnFrameId("canTransform argument sourceFrame", sourceFrame))
        return false;

    boost::mutex::scoped_lock lock(frameMutex);

    uint32_t targetId = lookupFrameNumber(targetFrame);
    uint32_t sourceId = lookupFrameNumber(sourceFrame);

    return canTransformNoLock(targetId, sourceId, time);
}

bool FrameTreeSimple::canTransform(const std::string& targetFrame,
        const boost::posix_time::ptime& targetTime, const std::string& sourceFrame,
        const boost::posix_time::ptime& sourceTime, const std::string& fixedFrame) const {
    if (warnFrameId("canTransform argument targetFrame", targetFrame))
        return false;
    if (warnFrameId("canTransform argument sourceFrame", sourceFrame))
        return false;
    if (warnFrameId("canTransform argument fixedFrame", fixedFrame))
        return false;

    return canTransform(targetFrame, fixedFrame, targetTime)
            && canTransform(fixedFrame, sourceFrame, sourceTime);

}

vector<string> FrameTreeSimple::getFrameStrings() const {
    vector<string> vec;

    boost::mutex::scoped_lock lock(frameMutex);

    TransformStorage temp;

    for (unsigned int counter = 1; counter < frameIDsReverse.size(); counter++) {
        vec.push_back(frameIDsReverse[counter]);
    }
    return vec;
}

bool FrameTreeSimple::frameExists(const string& frame_id_str) const {
    boost::mutex::scoped_lock lock(frameMutex);
    return frameIDs.count(frame_id_str);
}

string FrameTreeSimple::getParent(const string& frameId,
        const boost::posix_time::ptime& time) const {
    boost::mutex::scoped_lock lock(frameMutex);
    uint32_t frameNumber = lookupFrameNumber(frameId);
    FrameHistoryPtr frame = getFrame(frameNumber);

    if (!frame)
        throw LookupException("unknown frame number");

    uint32_t parentId = frame->getParent(time);
    if (parentId == 0)
        throw LookupException("has no parent");

    return lookupFrameString(parentId);
}

string FrameTreeSimple::allFramesAsDot() const {

    boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::local_time();

    std::stringstream mstream;
    mstream << "digraph G {" << std::endl;
    boost::mutex::scoped_lock lock(frameMutex);

    if (frames.size() == 1) {
        mstream << "\"no rct data recieved\"";
    }
    mstream.precision(3);
    mstream.setf(std::ios::fixed, std::ios::floatfield);

    for (unsigned int counter = 1; counter < frames.size(); counter++) // one referenced for 0 is no frame
            {
        unsigned int frame_id_num;
        FrameHistoryPtr counter_frame = getFrame(counter);
        if (!counter_frame) {
            continue;
        }
        try {
            frame_id_num = counter_frame->getData(boost::posix_time::from_time_t(0)).frameParent;
        } catch (RctException &e) {
            continue;
        }
        std::string authority = "no recorded authority";
        std::map<unsigned int, std::string>::const_iterator it = authorities.find(counter);
        if (it != authorities.end())
            authority = it->second;

        double rate =
                counter_frame->getListLength()
                        / std::max(
                                (counter_frame->getLatestTime() - counter_frame->getOldestTime()).total_milliseconds()
                                        / 1000.0, 0.0001);

        boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
        mstream << std::fixed;
        mstream.precision(3);
        mstream << "\"" << frameIDsReverse[frame_id_num] << "\"" << " -> " << "\""
                << frameIDsReverse[counter] << "\"" << "[label=\"" << "Publisher: " << authority
                << "\\n" << "Average rate: " << rate << " Hz\\n" << "Most recent transform: "
                << (counter_frame->getLatestTime() - epoch).total_milliseconds() / 1000.0 << " ";
        if (!currentTime.is_not_a_date_time())
            mstream << "( "
                    << (currentTime - counter_frame->getLatestTime()).total_milliseconds() / 1000.0
                    << " sec old)";
        mstream << "\\n" << "Buffer length: "
                << (counter_frame->getLatestTime() - counter_frame->getOldestTime()).total_milliseconds()
                        / 1000.0 << " sec\\n" << "\"];" << std::endl;
    }

    for (unsigned int counter = 1; counter < frames.size(); counter++) //one referenced for 0 is no frame
            {
        unsigned int frame_id_num;
        FrameHistoryPtr counter_frame = getFrame(counter);
        if (!counter_frame) {
            if (!currentTime.is_not_a_date_time()) {
                mstream << "edge [style=invis];" << std::endl;
                mstream
                        << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
                        << "\"Recorded at time: " << currentTime << "\"[ shape=plaintext ] ;\n "
                        << "}" << "->" << "\"" << frameIDsReverse[counter] << "\";" << std::endl;
            }
            continue;
        }
        try {

            frame_id_num = counter_frame->getData(boost::posix_time::from_time_t(0)).frameParent;
        } catch (RctException &e) {
            frame_id_num = 0;
        }

        if (frameIDsReverse[frame_id_num] == "NO_PARENT") {
            mstream << "edge [style=invis];" << std::endl;
            mstream
                    << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n";
            if (!currentTime.is_not_a_date_time())
                mstream << "\"Recorded at time: " << currentTime << "\"[ shape=plaintext ] ;\n ";
            mstream << "}" << "->" << "\"" << frameIDsReverse[counter] << "\";" << std::endl;
        }
    }
    mstream << "}";
    return mstream.str();
}

string FrameTreeSimple::allFramesAsYAML() const {
    boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::local_time();
    std::stringstream mstream;
    boost::mutex::scoped_lock lock(frameMutex);

    TransformStorage temp;

    if (frames.size() == 1)
        mstream << "[]";

    mstream.precision(3);
    mstream.setf(std::ios::fixed, std::ios::floatfield);

    for (unsigned int counter = 1; counter < frames.size(); counter++) //one referenced for 0 is no frame
            {
        uint32_t cfid = counter;
        uint32_t frameIdNum;
        FrameHistoryPtr cache = getFrame(cfid);
        if (!cache) {
            continue;
        }

        try {
            frameIdNum = cache->getData(boost::posix_time::from_time_t(0)).frameParent;
        } catch (RctException &e) {
            continue;
        }

        std::string authority = "no recorded authority";
        std::map<uint32_t, std::string>::const_iterator it = authorities.find(cfid);
        if (it != authorities.end()) {
            authority = it->second;
        }

        double rate = cache->getListLength()
                / std::max(
                        (cache->getLatestTime() - cache->getOldestTime()).total_milliseconds()
                                / 1000.0, 0.0001);
        boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
        mstream << std::fixed; //fixed point notation
        mstream.precision(3); //3 decimal places
        mstream << frameIDsReverse[cfid] << ": " << std::endl;
        mstream << "  parent: '" << frameIDsReverse[frameIdNum] << "'" << std::endl;
        mstream << "  broadcaster: '" << authority << "'" << std::endl;
        mstream << "  rate: " << rate << std::endl;
        mstream << "  most_recent_transform: "
                << (cache->getLatestTime() - epoch).total_milliseconds() / 1000.0 << std::endl;
        mstream << "  oldest_transform: "
                << (cache->getOldestTime() - epoch).total_milliseconds() / 1000.0 << std::endl;
        if (!currentTime.is_not_a_date_time()) {
            mstream << "  transform_delay: "
                    << (currentTime - cache->getLatestTime()).total_milliseconds() / 1000.0
                    << std::endl;
        }
        mstream << "  buffer_length: "
                << (cache->getLatestTime() - cache->getOldestTime()).total_milliseconds()
                << std::endl;
    }

    return mstream.str();
}

string FrameTreeSimple::allFramesAsString() const {
    boost::mutex::scoped_lock lock(frameMutex);
    return this->allFramesAsStringNoLock();
}

string FrameTreeSimple::allFramesAsStringNoLock() const {
    std::stringstream mstream;

    TransformStorage temp;

    for (unsigned int counter = 1; counter < frames.size(); counter++) {
        FrameHistoryPtr frame = getFrame(counter);
        if (frame == NULL)
            continue;
        uint32_t frameIdNum;
        try {
            frameIdNum = frame->getData(boost::posix_time::from_time_t(0)).frameParent;
        } catch (RctException &e) {
            frameIdNum = 0;
        }
        mstream << "Frame " << frameIDsReverse[counter] << " exists with parent "
                << frameIDsReverse[frameIdNum] << "." << std::endl;
    }

    return mstream.str();
}

FrameHistoryPtr FrameTreeSimple::getFrame(uint32_t frameId) const {
    if (frameId >= frames.size())
        return FrameHistoryPtr();
    else {
        return frames[frameId];
    }
}

FrameHistoryPtr FrameTreeSimple::allocateFrame(uint32_t cfid, bool isStatic) {
    FrameHistoryPtr frame = frames[cfid];
    if (isStatic) {
        frames[cfid] = FrameHistoryPtr(new StaticFrameHistory());
    } else {
        frames[cfid] = FrameHistoryPtr(new DynamicFrameHistory(cacheTime));
    }

    return frames[cfid];
}

uint32_t FrameTreeSimple::lookupOrInsertFrameNumber(const std::string& frameStr) {
    uint32_t retval = 0;
    boost::unordered_map<std::string, uint32_t>::iterator map_it = frameIDs.find(frameStr);
    if (map_it == frameIDs.end()) {
        retval = uint32_t(frames.size());
        frames.push_back(FrameHistoryPtr());
        frameIDs[frameStr] = retval;
        frameIDsReverse.push_back(frameStr);
    } else
        retval = frameIDs[frameStr];

    return retval;
}

uint32_t FrameTreeSimple::lookupFrameNumber(const std::string& frameStr) const {
    uint32_t retval;
    boost::unordered_map<std::string, uint32_t>::const_iterator map_it = frameIDs.find(frameStr);
    if (map_it == frameIDs.end()) {
        retval = uint32_t(0);
    } else
        retval = map_it->second;
    return retval;
}

const std::string& FrameTreeSimple::lookupFrameString(uint32_t frameNum) const {
    if (frameNum >= frameIDsReverse.size()) {
        std::stringstream ss;
        ss << "Reverse lookup of frame id " << frameNum << " failed!";
        throw LookupException(ss.str());
    } else
        return frameIDsReverse[frameNum];
}

uint32_t FrameTreeSimple::validateFrameId(const string& function, const string& frameId) const {
    if (frameId.empty()) {
        std::stringstream ss;
        ss << "Invalid argument passed to " << function << " in rct frame_ids cannot be empty";
        RSCERROR(logger, ss.str());
        throw InvalidArgumentException(ss.str());
    }

    if (boost::algorithm::starts_with(frameId, "/")) {
        std::stringstream ss;
        ss << "Invalid argument \"" << frameId << "\" passed to " << function
                << " in rct frame ids cannot start with a '/' like: ";
        RSCERROR(logger, ss.str());
        throw InvalidArgumentException(ss.str().c_str());
    }

    uint32_t id = lookupFrameNumber(frameId);
    if (id == 0) {
        std::stringstream ss;
        ss << "\"" << frameId << "\" passed to " << function << " does not exist. ";
        RSCERROR(logger, ss.str());
        throw LookupException(ss.str());
    }

    return id;
}

bool FrameTreeSimple::warnFrameId(const string& function, const string& frameId) const {
    if (frameId.empty()) {
        std::stringstream ss;
        ss << "Invalid argument passed to " << function << " in rct frame_ids cannot be empty";
        RSCWARN(logger, ss.str());
        return true;
    }

    if (boost::algorithm::starts_with(frameId, "/")) {
        std::stringstream ss;
        ss << "Invalid argument \"" << frameId << "\" passed to " << function
                << " in rct frame ids cannot start with a '/' like: ";
        RSCWARN(logger, ss.str());
        return true;
    }

    return false;
}

struct TimeAndFrameIDFrameComparator {
    TimeAndFrameIDFrameComparator(uint32_t id) :
            id(id) {
    }

    bool operator()(const pair<boost::posix_time::ptime, uint32_t>& rhs) const {
        return rhs.second == id;
    }

    uint32_t id;
};

boost::posix_time::ptime FrameTreeSimple::getLatestCommonTime(uint32_t targetId,
        uint32_t sourceId) const {
    // Error if one of the frames don't exist.
    if (sourceId == 0 || targetId == 0){
        RSCERROR(logger, "ids are 0");
        throw LookupException("ids are 0");
    }

    if (sourceId == targetId) {
        FrameHistoryPtr cache = getFrame(sourceId);
        //Set time to latest timestamp of frameid in case of target and source frame id are the same
        if (cache)
            return cache->getLatestTime();
        else
            return boost::posix_time::from_time_t(0);
    }

    std::vector<pair<boost::posix_time::ptime, uint32_t> > lctCache;

    // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
    // in the target is a direct parent
    uint32_t frame = sourceId;
    pair<boost::posix_time::ptime, uint32_t> temp;
    uint32_t depth = 0;
    boost::posix_time::ptime commonTime(boost::posix_time::pos_infin);
    while (frame != 0) {
        FrameHistoryPtr cache = getFrame(frame);

        if (!cache) {
            // There will be no cache for the very root of the tree
            break;
        }

        pair<boost::posix_time::ptime, uint32_t> latest = cache->getLatestTimeAndParent();

        if (latest.second == 0) {
            // Just break out here... there may still be a path from source -> target
            break;
        }

        if (!latest.first.is_not_a_date_time()) {
            commonTime = std::min(latest.first, commonTime);
        }

        lctCache.push_back(latest);

        frame = latest.second;

        // Early out... target frame is a direct parent of the source frame
        if (frame == targetId) {
            if (commonTime.is_pos_infinity()) {
                return boost::posix_time::from_time_t(0);
            }

            return commonTime;
        }

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {

            std::stringstream ss;
            ss << "The rct tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            RSCERROR(logger, ss.str());
            throw LookupException(ss.str());
        }
    }

    // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
    frame = targetId;
    depth = 0;
    commonTime = boost::posix_time::ptime(boost::posix_time::pos_infin);
    uint32_t commonParent = 0;
    while (true) {
        FrameHistoryPtr cache = getFrame(frame);

        if (!cache) {
            break;
        }

        pair<boost::posix_time::ptime, uint32_t> latest = cache->getLatestTimeAndParent();

        if (latest.second == 0) {
            break;
        }

        if (!latest.first.is_not_a_date_time()) {
            commonTime = std::min(latest.first, commonTime);
        }

        std::vector<pair<boost::posix_time::ptime, uint32_t> >::iterator it = std::find_if(
                lctCache.begin(), lctCache.end(), TimeAndFrameIDFrameComparator(latest.second));
        if (it != lctCache.end()) // found a common parent
                {
            commonParent = it->second;
            break;
        }

        frame = latest.second;

        // Early out... source frame is a direct parent of the target frame
        if (frame == sourceId) {
            if (commonTime.is_pos_infinity()) {
                return boost::posix_time::from_time_t(0);
            }
            return commonTime;
        }

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {
            std::stringstream ss;
            ss << "The tf tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            RSCERROR(logger, ss.str());
            throw LookupException(ss.str());
        }
    }

    if (commonParent == 0) {
        string err(
                "Could not find a connection between '" + lookupFrameString(targetId) + "' and '"
                        + lookupFrameString(sourceId)
                        + "' because they are not part of the same tree."
                        + "Rct has two or more unconnected trees.");
        RSCERROR(logger, err);
        throw ConnectivityException(err);
    }

    {
        std::vector<pair<boost::posix_time::ptime, uint32_t> >::iterator it = lctCache.begin();
        std::vector<pair<boost::posix_time::ptime, uint32_t> >::iterator end = lctCache.end();
        for (; it != end; ++it) {
            if (!it->first.is_not_a_date_time()) {
                commonTime = std::min(commonTime, it->first);
            }

            if (it->second == commonParent) {
                break;
            }
        }
    }

    if (commonTime.is_pos_infinity()) {
        commonTime = boost::posix_time::from_time_t(0);
    }

    return commonTime;
}

boost::signals2::connection FrameTreeSimple::addTransformsChangedListener(
        boost::function<void(void)> callback) {
    boost::mutex::scoped_lock lock(requestsMutex);
    return transformsChangedSignal.connect(callback);
}

void FrameTreeSimple::removeTransformsChangedListener(boost::signals2::connection c) {
    boost::mutex::scoped_lock lock(requestsMutex);
    c.disconnect();
}

} /* namespace rct */
