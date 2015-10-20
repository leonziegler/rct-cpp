/*
 * FrameTreeSimple.cpp
 *
 *  Created on: Oct 19, 2015
 *      Author: leon
 */

#include "FrameTreeSimple.h"

namespace rct {

FrameTreeSimple::FrameTreeSimple(const boost::posix_time::time_duration& cacheTime):cacheTime(cacheTime) {
}

FrameTreeSimple::~FrameTreeSimple() {
}

void FrameTreeSimple::clear() {
    boost::mutex::scoped_lock lock(frameMutex);
      if ( frames_.size() > 1 )
      {
        for (std::vector<TimeCacheInterfacePtr>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
        {
          if (*cache_it)
            (*cache_it)->clearList();
        }
      }
}

bool FrameTreeSimple::setTransform(const Transform& transform, bool is_static) {
}

Transform FrameTreeSimple::lookupTransform(const std::string& target_frame,
        const std::string& source_frame, const boost::posix_time::ptime& time) const {
}

Transform FrameTreeSimple::lookupTransform(const std::string& target_frame,
        const boost::posix_time::ptime& target_time, const std::string& source_frame,
        const boost::posix_time::ptime& source_time, const std::string& fixed_frame) const {
}

bool FrameTreeSimple::canTransform(const std::string& target_frame, const std::string& source_frame,
        const boost::posix_time::ptime& time, std::string* error_msg) const {
}

bool FrameTreeSimple::canTransform(const std::string& target_frame,
        const boost::posix_time::ptime& target_time, const std::string& source_frame,
        const boost::posix_time::ptime& source_time, const std::string& fixed_frame,
        std::string* error_msg) const {
}

std::vector<std::string> FrameTreeSimple::getFrameStrings() const {
}

bool FrameTreeSimple::frameExists(const std::string& frame_id_str) const {
}

std::string FrameTreeSimple::getParent(const std::string& frame_id,
        const boost::posix_time::ptime& time) const {
}

std::string FrameTreeSimple::allFramesAsDot() const {
}

std::string FrameTreeSimple::allFramesAsYAML() const {
}

std::string FrameTreeSimple::allFramesAsString() const {
}

boost::signals2::connection FrameTreeSimple::addTransformsChangedListener(
        boost::function<void(void)> callback) {
}

void FrameTreeSimple::removeTransformsChangedListener(boost::signals2::connection c) {
}

} /* namespace rct */
