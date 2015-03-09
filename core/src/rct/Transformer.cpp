/*
 * Transformer.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include "Transformer.h"

using namespace std;

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

string Transformer::getAuthorityName() const {
	return comm->getAuthorityName();
}

bool Transformer::sendTransform(const Transform& transform, TransformType type) {
	return comm->sendTransform(transform, type);
}

bool Transformer::sendTransform(const std::vector<Transform>& transforms, TransformType type) {
	return comm->sendTransform(transforms, type);
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

Transformer::FuturePtr Transformer::requestTransform(const std::string& target_frame,
			const std::string& source_frame,
			const boost::posix_time::ptime& time) {
	return core->requestTransform(target_frame, source_frame, time);
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
