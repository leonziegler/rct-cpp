/*
 * Transformer.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include "Transformer.h"

namespace rct {



Transformer::Transformer() {
}

Transformer::~Transformer() {
}

bool Transformer::sendTransform(const Transform& transform) {
}

bool Transformer::sendTransform(const std::vector<Transform>& transforms) {
}

Transform Transformer::lookupTransform(const std::string& target_frame,
		const std::string& source_frame,
		const boost::posix_time::ptime& time) const {
}

Transform Transformer::lookupTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
}

bool Transformer::canTransform(const std::string& target_frame,
		const std::string& source_frame, const boost::posix_time::ptime& time,
		std::string* error_msg) const {
}

bool Transformer::canTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame, std::string* error_msg) const {
}

}  // namespace rct
