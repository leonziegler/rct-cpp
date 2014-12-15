/*
 * Transformer.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include <Transformer.h>

rct::Transformer::Transformer() {
}

rct::Transformer::~Transformer() {
}

bool rct::Transformer::sendTransform(const Transform& transform) {
}

bool rct::Transformer::sendTransform(const std::vector<Transform>& transforms) {
}

Transform rct::Transformer::lookupTransform(const std::string& target_frame,
		const std::string& source_frame,
		const boost::posix_time::ptime& time) const {
}

Transform rct::Transformer::lookupTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
}

bool rct::Transformer::canTransform(const std::string& target_frame,
		const std::string& source_frame, const boost::posix_time::ptime& time,
		std::string* error_msg) const {
}

bool rct::Transformer::canTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame, std::string* error_msg) const {
}
