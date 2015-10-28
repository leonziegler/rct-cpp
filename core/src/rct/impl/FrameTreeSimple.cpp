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
        for (std::vector<FrameHistoryPtr>::iterator cache_it = frames.begin() + 1;
                cache_it != frames.end(); ++cache_it) {
            if (*cache_it)
                (*cache_it)->clearList();
        }
    }
}

bool FrameTreeSimple::setTransform(const Transform& transform, bool is_static) {
    Transform stripped = transform;
    stripped.setFrameParent(
            boost::algorithm::trim_left_copy_if(stripped.getFrameParent(), boost::is_any_of("/")));
    stripped.setFrameChild(
            boost::algorithm::trim_left_copy_if(stripped.getFrameChild(), boost::is_any_of("/")));

    bool error_exists = false;
    if (stripped.getFrameChild() == stripped.getFrameParent()) {
        RSCERROR(this->logger,
                "Self transform: Ignoring transform from authority \"" << stripped.getAuthority() << "\" with parent and child frame \"" << stripped.getFrameChild() << "\" because they are the same");
        error_exists = true;
    }

    if (stripped.getFrameChild() == "") {
        RSCERROR(this->logger,
                "No child name: Ignoring transform from authority \"" << stripped.getAuthority() << "\" because child frame not set");
        error_exists = true;
    }

    if (stripped.getFrameParent() == "") {
        RSCERROR(this->logger,
                "No parent frame: Ignoring transform with child frame \"" << stripped.getFrameChild() << "\"  from authority \"" << stripped.getAuthority() << "\" because parent frame is not set");
        error_exists = true;
    }

    if (error_exists)
        return false;

    {
        boost::mutex::scoped_lock lock(frameMutex);
        uint32_t frame_number = lookupOrInsertFrameNumber(stripped.getFrameChild());
        FrameHistoryPtr frame = getFrame(frame_number);
        if (frame == NULL)
            frame = allocateFrame(frame_number, is_static);

        TransformStorage storage;
        storage.transform = stripped.getTransform();
        storage.frameParent = lookupOrInsertFrameNumber(stripped.getFrameParent());
        storage.frameChild = frame_number;
        if (frame->insertData(storage)) {
            authorities[frame_number] = stripped.getAuthority();
        } else {
            RSCWARN(this->logger,
                    "Old data: Ignoring data from the past for frame " << stripped.getFrameChild() << " at time " << stripped.getTime() <<" according to authority " << stripped.getAuthority() <<"\nPossible reasons: Data is older than than allowed by the cache time parameter, the component receives outdated transformation information, something bad is going on with the system/simulation time");
            return false;
        }
    }

    // TODO: testTransformableRequests()?;

    return true;
}

enum WalkEnding {
    Identity, TargetParentOfSource, SourceParentOfTarget, FullPath,
};

struct TransformAccum {
    TransformAccum() :
                    source_to_top_quat(0.0, 0.0, 0.0, 1.0),
                    source_to_top_vec(0.0, 0.0, 0.0),
                    target_to_top_quat(0.0, 0.0, 0.0, 1.0),
                    target_to_top_vec(0.0, 0.0, 0.0),
                    result_quat(0.0, 0.0, 0.0, 1.0),
                    result_vec(0.0, 0.0, 0.0) {
    }

    uint32_t gather(FrameHistoryPtr cache, boost::posix_time::ptime time) {
        return cache->getData(time).frameParent;
    }

    void accum(bool source) {
        if (source) {
            source_to_top_vec = st.transform.rotation() * source_to_top_vec
                    + st.transform.translation();
            source_to_top_quat = st.transform.rotation() * source_to_top_quat;
        } else {
            target_to_top_vec = st.transform.rotation() * target_to_top_vec
                    + st.transform.translation();
            target_to_top_quat = st.transform.rotation() * target_to_top_quat;
        }
    }

    void finalize(WalkEnding end, boost::posix_time::ptime _time) {
        switch (end) {
        case Identity:
            break;
        case TargetParentOfSource:
            result_vec = source_to_top_vec;
            result_quat = source_to_top_quat;
            break;
        case SourceParentOfTarget: {
            Eigen::Quaterniond inv_target_quat = target_to_top_quat.inverse();
            Eigen::Vector3d inv_target_vec = inv_target_quat * -target_to_top_vec;
            result_vec = inv_target_vec;
            result_quat = inv_target_quat;
            break;
        }
        case FullPath: {
            Eigen::Quaterniond inv_target_quat = target_to_top_quat.inverse();
            Eigen::Vector3d inv_target_vec = inv_target_quat * -target_to_top_vec;

            result_vec = inv_target_quat * source_to_top_vec + inv_target_vec;
            result_quat = inv_target_quat * source_to_top_quat;
        }
            break;
        };

        time = _time;
    }

    TransformStorage st;
    boost::posix_time::ptime time;
    Eigen::Quaterniond source_to_top_quat;
    Eigen::Vector3d source_to_top_vec;
    Eigen::Quaterniond target_to_top_quat;
    Eigen::Vector3d target_to_top_vec;

    Eigen::Quaterniond result_quat;
    Eigen::Vector3d result_vec;
};

struct CanTransformAccum {
    uint32_t gather(FrameHistoryPtr cache, boost::posix_time::ptime time) {
        return cache->getParent(time);
    }

    void accum(bool source) {
    }

    void finalize(WalkEnding end, boost::posix_time::ptime _time) {
    }

    TransformStorage st;
};

template<typename F>
void FrameTreeSimple::walkToTopParent(F& f, boost::posix_time::ptime time, uint32_t target_id,
        uint32_t source_id, std::vector<uint32_t> *frame_chain) const {
    if (frame_chain)
        frame_chain->clear();

    // Short circuit if zero length transform to allow lookups on non existant links
    if (source_id == target_id) {
        f.finalize(Identity, time);
        return;
    }

    //If getting the latest get the latest common time
    if (time.is_not_a_date_time() || time == boost::posix_time::from_time_t(0)) {
        time = getLatestCommonTime(target_id, source_id);
    }

    // Walk the tree to its root from the source frame, accumulating the transform
    uint32_t frame = source_id;
    uint32_t top_parent = frame;
    uint32_t depth = 0;

    std::string extrapolation_error_string;
    bool extrapolation_might_have_occurred = false;

    while (frame != 0) {
        FrameHistoryPtr cache = getFrame(frame);
        if (frame_chain)
            frame_chain->push_back(frame);

        if (!cache) {
            // There will be no cache for the very root of the tree
            top_parent = frame;
            break;
        }

        uint32_t parent = f.gather(cache, time);
        if (parent == 0) {
            // Just break out here... there may still be a path from source -> target
            top_parent = frame;
            extrapolation_might_have_occurred = true;
            break;
        }

        // Early out... target frame is a direct parent of the source frame
        if (frame == target_id) {
            f.finalize(TargetParentOfSource, time);
            return;
        }

        f.accum(true);

        top_parent = frame;
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
    frame = target_id;
    depth = 0;
    std::vector<uint32_t> reverse_frame_chain;

    while (frame != top_parent) {
        FrameHistoryPtr cache = getFrame(frame);
        if (frame_chain)
            reverse_frame_chain.push_back(frame);

        if (!cache) {
            break;
        }

        uint32_t parent = f.gather(cache, time);
        if (parent == 0) {

            std::stringstream ss;
            ss << "Extrapolation error when looking up transform from frame ["
                    << lookupFrameString(source_id) << "] to frame ["
                    << lookupFrameString(target_id) << "]";
            throw ExtrapolationException(ss.str());
        }

        // Early out... source frame is a direct parent of the target frame
        if (frame == source_id) {
            f.finalize(SourceParentOfTarget, time);
            if (frame_chain) {
                frame_chain->swap(reverse_frame_chain);
            }
            return;
        }

        f.accum(false);

        frame = parent;

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {

            std::stringstream ss;
            ss << "The rct tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            throw LookupException(ss.str());
        }
    }

    if (frame != top_parent) {
        if (extrapolation_might_have_occurred) {
            std::stringstream ss;
            ss << "Extrapolation error when looking up transform from frame ["
                    << lookupFrameString(source_id) << "] to frame ["
                    << lookupFrameString(target_id) << "]";
            throw ExtrapolationException(ss.str());

        }

        string err(
                "Could not find a connection between '" + lookupFrameString(target_id) + "' and '"
                        + lookupFrameString(source_id)
                        + "' because they are not part of the same tree."
                        + "Rct has two or more unconnected trees.");
        throw ConnectivityException(err);

        f.finalize(FullPath, time);
        if (frame_chain) {
            // Pruning: Compare the chains starting at the parent (end) until they differ
            int m = reverse_frame_chain.size() - 1;
            int n = frame_chain->size() - 1;
            for (; m >= 0 && n >= 0; --m, --n) {
                if ((*frame_chain)[n] != reverse_frame_chain[m])
                    break;
            }
            // Erase all duplicate items from frame_chain
            if (n > 0)
                frame_chain->erase(frame_chain->begin() + (n - 1), frame_chain->end());

            if (m < reverse_frame_chain.size()) {
                for (int i = m; i >= 0; --i) {
                    frame_chain->push_back(reverse_frame_chain[i]);
                }
            }
        }
    }
}

Transform FrameTreeSimple::lookupTransform(const std::string& target_frame,
        const std::string& source_frame, const boost::posix_time::ptime& time) const {
    boost::mutex::scoped_lock lock(frameMutex);

    if (target_frame == source_frame) {
        Eigen::Affine3d transIdent;
        transIdent.setIdentity();
        Transform identity(transIdent, target_frame, source_frame, time);

        if (time.is_not_a_date_time() || time == boost::posix_time::from_time_t(0)) {
            uint32_t target_id = lookupFrameNumber(target_frame);
            FrameHistoryPtr cache = getFrame(target_id);
            if (cache)
                identity.setTime(cache->getLatestTime());
        }

        return identity;
    }

    //Identify case does not need to be validated above
    uint32_t target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
    uint32_t source_id = validateFrameId("lookupTransform argument source_frame", source_frame);

    std::string error_string;
    TransformAccum accum;
    walkToTopParent(accum, time, target_id, source_id, NULL);

    Eigen::Affine3d trans;
    trans.fromPositionOrientationScale(accum.result_vec, accum.result_quat,
            Eigen::Vector3d::Ones());
    Transform output_transform(trans, target_frame, source_frame, accum.time);
    return output_transform;
}

Transform FrameTreeSimple::lookupTransform(const std::string& target_frame,
        const boost::posix_time::ptime& target_time, const std::string& source_frame,
        const boost::posix_time::ptime& source_time, const std::string& fixed_frame) const {
    validateFrameId("lookupTransform argument target_frame", target_frame);
    validateFrameId("lookupTransform argument source_frame", source_frame);
    validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

    Transform output;
    Transform temp1 = lookupTransform(fixed_frame, source_frame, source_time);
    Transform temp2 = lookupTransform(target_frame, fixed_frame, target_time);

    output.setTransform(temp2.getTransform() * temp1.getTransform());
    output.setTime(temp2.getTime());
    output.setFrameParent(target_frame);
    output.setFrameChild(source_frame);
    return output;
}

bool FrameTreeSimple::canTransformNoLock(uint32_t target_id, uint32_t source_id,
        const boost::posix_time::ptime& time) const {
    if (target_id == 0 || source_id == 0) {
        return false;
    }

    if (target_id == source_id) {
        return true;
    }

    CanTransformAccum accum;
    try {
        walkToTopParent(accum, time, target_id, source_id, NULL);
    } catch (RctException &e) {
        RSCDEBUG(logger, "Error while checking if transform is available: " << e.what());
        return false;
    }

    return true;
}

bool FrameTreeSimple::canTransformInternal(uint32_t target_id, uint32_t source_id,
        const boost::posix_time::ptime& time) const {
    boost::mutex::scoped_lock lock(frameMutex);
    return canTransformNoLock(target_id, source_id, time);
}

bool FrameTreeSimple::canTransform(const std::string& target_frame, const std::string& source_frame,
        const boost::posix_time::ptime& time) const {
    // Short circuit if target_frame == source_frame
    if (target_frame == source_frame)
        return true;

    if (warnFrameId("canTransform argument target_frame", target_frame))
        return false;
    if (warnFrameId("canTransform argument source_frame", source_frame))
        return false;

    boost::mutex::scoped_lock lock(frameMutex);

    uint32_t target_id = lookupFrameNumber(target_frame);
    uint32_t source_id = lookupFrameNumber(source_frame);

    return canTransformNoLock(target_id, source_id, time);
}

bool FrameTreeSimple::canTransform(const std::string& target_frame,
        const boost::posix_time::ptime& target_time, const std::string& source_frame,
        const boost::posix_time::ptime& source_time, const std::string& fixed_frame) const {
    if (warnFrameId("canTransform argument target_frame", target_frame))
        return false;
    if (warnFrameId("canTransform argument source_frame", source_frame))
        return false;
    if (warnFrameId("canTransform argument fixed_frame", fixed_frame))
        return false;

    return canTransform(target_frame, fixed_frame, target_time)
            && canTransform(fixed_frame, source_frame, source_time);

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

string FrameTreeSimple::getParent(const string& frame_id,
        const boost::posix_time::ptime& time) const {
    boost::mutex::scoped_lock lock(frameMutex);
    uint32_t frame_number = lookupFrameNumber(frame_id);
    FrameHistoryPtr frame = getFrame(frame_number);

    if (!frame)
        throw LookupException("unknown frame number");

    uint32_t parent_id = frame->getParent(time);
    if (parent_id == 0)
        throw LookupException("has no parent");

    return lookupFrameString(parent_id);
}

string FrameTreeSimple::allFramesAsDot() const {

    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();

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
        if (!current_time.is_not_a_date_time())
            mstream << "( "
                    << (current_time - counter_frame->getLatestTime()).total_milliseconds() / 1000.0
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
            if (!current_time.is_not_a_date_time()) {
                mstream << "edge [style=invis];" << std::endl;
                mstream
                        << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
                        << "\"Recorded at time: " << current_time << "\"[ shape=plaintext ] ;\n "
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
            if (!current_time.is_not_a_date_time())
                mstream << "\"Recorded at time: " << current_time << "\"[ shape=plaintext ] ;\n ";
            mstream << "}" << "->" << "\"" << frameIDsReverse[counter] << "\";" << std::endl;
        }
    }
    mstream << "}";
    return mstream.str();
}

string FrameTreeSimple::allFramesAsYAML() const {
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
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
        uint32_t frame_id_num;
        FrameHistoryPtr cache = getFrame(cfid);
        if (!cache) {
            continue;
        }

        try {
            frame_id_num = cache->getData(boost::posix_time::from_time_t(0)).frameParent;
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
        mstream << "  parent: '" << frameIDsReverse[frame_id_num] << "'" << std::endl;
        mstream << "  broadcaster: '" << authority << "'" << std::endl;
        mstream << "  rate: " << rate << std::endl;
        mstream << "  most_recent_transform: "
                << (cache->getLatestTime() - epoch).total_milliseconds() / 1000.0 << std::endl;
        mstream << "  oldest_transform: "
                << (cache->getOldestTime() - epoch).total_milliseconds() / 1000.0 << std::endl;
        if (!current_time.is_not_a_date_time()) {
            mstream << "  transform_delay: "
                    << (current_time - cache->getLatestTime()).total_milliseconds() / 1000.0
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
        FrameHistoryPtr frame_ptr = getFrame(counter);
        if (frame_ptr == NULL)
            continue;
        uint32_t frame_id_num;
        try {
            frame_id_num = frame_ptr->getData(boost::posix_time::from_time_t(0)).frameParent;
        } catch (RctException &e) {
            frame_id_num = 0;
        }
        mstream << "Frame " << frameIDsReverse[counter] << " exists with parent "
                << frameIDsReverse[frame_id_num] << "." << std::endl;
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
    FrameHistoryPtr frame_ptr = frames[cfid];
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

uint32_t FrameTreeSimple::validateFrameId(const string& function, const string& frame_id) const {
    if (frame_id.empty()) {
        std::stringstream ss;
        ss << "Invalid argument passed to " << function << " in rct frame_ids cannot be empty";
        throw InvalidArgumentException(ss.str());
    }

    if (boost::algorithm::starts_with(frame_id, "/")) {
        std::stringstream ss;
        ss << "Invalid argument \"" << frame_id << "\" passed to " << function
                << " in rct frame ids cannot start with a '/' like: ";
        throw InvalidArgumentException(ss.str().c_str());
    }

    uint32_t id = lookupFrameNumber(frame_id);
    if (id == 0) {
        std::stringstream ss;
        ss << "\"" << frame_id << "\" passed to " << function << " does not exist. ";
        throw LookupException(ss.str());
    }

    return id;
}

bool FrameTreeSimple::warnFrameId(const string& function, const string& frame_id) const {
    if (frame_id.empty()) {
        std::stringstream ss;
        ss << "Invalid argument passed to " << function << " in rct frame_ids cannot be empty";
        RSCWARN(logger, ss.str());
        return true;
    }

    if (boost::algorithm::starts_with(frame_id, "/")) {
        std::stringstream ss;
        ss << "Invalid argument \"" << frame_id << "\" passed to " << function
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

boost::posix_time::ptime FrameTreeSimple::getLatestCommonTime(uint32_t target_id,
        uint32_t source_id) const {
    // Error if one of the frames don't exist.
    if (source_id == 0 || target_id == 0)
        throw LookupException("ids are 0");

    if (source_id == target_id) {
        FrameHistoryPtr cache = getFrame(source_id);
        //Set time to latest timestamp of frameid in case of target and source frame id are the same
        if (cache)
            return cache->getLatestTime();
        else
            return boost::posix_time::from_time_t(0);
    }

    std::vector<pair<boost::posix_time::ptime, uint32_t> > lct_cache;

    // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
    // in the target is a direct parent
    uint32_t frame = source_id;
    pair<boost::posix_time::ptime, uint32_t> temp;
    uint32_t depth = 0;
    boost::posix_time::ptime common_time(boost::posix_time::pos_infin);
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
            common_time = std::min(latest.first, common_time);
        }

        lct_cache.push_back(latest);

        frame = latest.second;

        // Early out... target frame is a direct parent of the source frame
        if (frame == target_id) {
            if (common_time.is_pos_infinity()) {
                return boost::posix_time::from_time_t(0);
            }

            return common_time;
        }

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {

            std::stringstream ss;
            ss << "The rct tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            throw LookupException(ss.str());
        }
    }

    // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
    frame = target_id;
    depth = 0;
    common_time = boost::posix_time::ptime(boost::posix_time::pos_infin);
    uint32_t common_parent = 0;
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
            common_time = std::min(latest.first, common_time);
        }

        std::vector<pair<boost::posix_time::ptime, uint32_t> >::iterator it = std::find_if(
                lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
        if (it != lct_cache.end()) // found a common parent
                {
            common_parent = it->second;
            break;
        }

        frame = latest.second;

        // Early out... source frame is a direct parent of the target frame
        if (frame == source_id) {
            if (common_time.is_pos_infinity()) {
                return boost::posix_time::from_time_t(0);
            }
            return common_time;
        }

        ++depth;
        if (depth > MAX_GRAPH_DEPTH) {
            std::stringstream ss;
            ss << "The tf tree is invalid because it contains a loop." << std::endl
                    << allFramesAsStringNoLock() << std::endl;
            throw LookupException(ss.str());
        }
    }

    if (common_parent == 0) {
        string err(
                "Could not find a connection between '" + lookupFrameString(target_id) + "' and '"
                        + lookupFrameString(source_id)
                        + "' because they are not part of the same tree."
                        + "Rct has two or more unconnected trees.");
        throw ConnectivityException(err);
    }

    {
        std::vector<pair<boost::posix_time::ptime, uint32_t> >::iterator it = lct_cache.begin();
        std::vector<pair<boost::posix_time::ptime, uint32_t> >::iterator end = lct_cache.end();
        for (; it != end; ++it) {
            if (!it->first.is_not_a_date_time()) {
                common_time = std::min(common_time, it->first);
            }

            if (it->second == common_parent) {
                break;
            }
        }
    }

    if (common_time.is_pos_infinity()) {
        common_time = boost::posix_time::from_time_t(0);
    }

    return common_time;
}

boost::signals2::connection FrameTreeSimple::addTransformsChangedListener(
        boost::function<void(void)> callback) {
    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    return _transforms_changed_.connect(callback);
}

void FrameTreeSimple::removeTransformsChangedListener(boost::signals2::connection c) {
    boost::mutex::scoped_lock lock(transformable_requests_mutex_);
    c.disconnect();
}

} /* namespace rct */
