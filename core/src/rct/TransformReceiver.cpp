/*
 * Transformer.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include "TransformReceiver.h"

using namespace std;

namespace rct {

TransformReceiver::TransformReceiver(const TransformerCore::Ptr &core,
		const TransformCommunicator::Ptr &comm, const TransformerConfig& conf) :
		core(core), comm(comm), config(conf) {
}

TransformReceiver::~TransformReceiver() {
}

void TransformReceiver::printContents(std::ostream& stream) const {
	stream << "core = {";
	core->printContents(stream);
	stream << "}, communicator = {";
	comm->printContents(stream);
	stream << "}, config = {";
	config.print(stream);
	stream << "}";
}

TransformerConfig TransformReceiver::getConfig() const {
	return config;
}

string TransformReceiver::getAuthorityName() const {
	return comm->getAuthorityName();
}

Transform TransformReceiver::lookupTransform(const std::string& target_frame,
		const std::string& source_frame,
		const boost::posix_time::ptime& time) const {
	return core->lookupTransform(target_frame, source_frame, time);
}

Transform TransformReceiver::lookupTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
	return core->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

TransformReceiver::FuturePtr TransformReceiver::requestTransform(const std::string& target_frame,
			const std::string& source_frame,
			const boost::posix_time::ptime& time) {
	return core->requestTransform(target_frame, source_frame, time);
}

bool TransformReceiver::canTransform(const std::string& target_frame,
		const std::string& source_frame, const boost::posix_time::ptime& time,
		std::string* error_msg) const {
	return core->canTransform(target_frame, source_frame, time, error_msg);
}

bool TransformReceiver::canTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame, std::string* error_msg) const {
	return core->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, error_msg);
}

void TransformReceiver::shutdown() {
	comm->shutdown();
}

}  // namespace rct
