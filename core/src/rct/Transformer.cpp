/*
 * Transformer.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include "Transformer.h"

namespace rct {

Transformer::Transformer(const TransformerCore::Ptr &core,
		const TransformCommunicator::Ptr &comm, const TransformerConfig& conf) :
		core(core), comm(comm), config(conf) {
}

Transformer::~Transformer() {
}

void Transformer::printContents(std::ostream& stream) const {
	stream << "core = {";
	core->printContents(stream);
	stream << "}, communicator = {";
	comm->printContents(stream);
	stream << "}, config = {";
	config.print(stream);
	stream << "}";
}

TransformerConfig Transformer::getConfig() const {
	return config;
}

bool Transformer::sendTransform(const Transform& transform) {
	return comm->sendTransform(transform);
}

bool Transformer::sendTransform(const std::vector<Transform>& transforms) {
	return comm->sendTransform(transforms);
}

Transform Transformer::lookupTransform(const std::string& target_frame,
		const std::string& source_frame,
		const boost::posix_time::ptime& time) const {
	return core->lookupTransform(target_frame, source_frame, time);
}

Transform Transformer::lookupTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
	return core->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

bool Transformer::canTransform(const std::string& target_frame,
		const std::string& source_frame, const boost::posix_time::ptime& time,
		std::string* error_msg) const {
	return core->canTransform(target_frame, source_frame, time, error_msg);
}

bool Transformer::canTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame, std::string* error_msg) const {
	return core->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, error_msg);
}

}  // namespace rct
