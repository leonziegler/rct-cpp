/*
 * Transformer.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include "TransformPublisher.h"

using namespace std;

namespace rct {

TransformPublisher::TransformPublisher(const TransformCommunicator::Ptr &comm, const TransformerConfig& conf) :
		comm(comm), config(conf) {
}

TransformPublisher::~TransformPublisher() {
}

void TransformPublisher::printContents(std::ostream& stream) const {
	stream << "communicator = {";
	comm->printContents(stream);
	stream << "}, config = {";
	config.print(stream);
	stream << "}";
}

TransformerConfig TransformPublisher::getConfig() const {
	return config;
}

string TransformPublisher::getAuthorityName() const {
	return comm->getAuthorityName();
}

bool TransformPublisher::sendTransform(const Transform& transform, TransformType type) {
	return comm->sendTransform(transform, type);
}

bool TransformPublisher::sendTransform(const std::vector<Transform>& transforms, TransformType type) {
	return comm->sendTransform(transforms, type);
}

void TransformPublisher::shutdown() {
	comm->shutdown();
}

}  // namespace rct
