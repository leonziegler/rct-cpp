/*
 * TransformerFactory.cpp
 *
 *  Created on: Dec 7, 2014
 *      Author: leon
 */

#include "TransformerFactory.h"
#include "rct/rctConfig.h"
#ifdef RCT_HAVE_TF2
#include "impl/TransformerTF2.h"
#endif
#ifdef RCT_HAVE_RSB
#include <rct/impl/TransformCommRsb.h>
#endif
#ifdef RCT_HAVE_ROS
#include <rct/impl/TransformCommRos.h>
#endif

using namespace std;

namespace rct {

TransformerFactory::TransformerFactory() {
}

TransformerFactory::~TransformerFactory() {
}

TransformerFactory& getTransformerFactory() {
	return TransformerFactory::getInstanceBase();
}

TransformerFactory& TransformerFactory::getInstanceBase() {
	return rsc::patterns::Singleton<TransformerFactory>::getInstance();
}

TransformReceiver::Ptr TransformerFactory::createTransformReceiver(const TransformerConfig& config) const {
	vector<TransformListener::Ptr> allListeners;
	return createTransformReceiver(allListeners, config);
}

TransformReceiver::Ptr TransformerFactory::createTransformReceiver(const TransformListener::Ptr& listener, const TransformerConfig& config) const {
	vector<TransformListener::Ptr> allListeners;
	allListeners.push_back(listener);
	return createTransformReceiver(allListeners, config);
}

TransformReceiver::Ptr TransformerFactory::createTransformReceiver(const vector<TransformListener::Ptr>& listeners, const TransformerConfig& config) const {
	vector<TransformListener::Ptr> allListeners;
	allListeners.insert(allListeners.end(), listeners.begin(), listeners.end());
	TransformerCore::Ptr core;

#ifdef RCT_HAVE_TF2
	core = TransformerTF2::Ptr(new TransformerTF2(config.getCacheTime()));
#else
	throw TransformerFactoryException("No known logic implementation available!");
#endif

	allListeners.push_back(core);

	// order is priority
	vector<TransformCommunicator::Ptr> comms;
#ifdef RCT_HAVE_RSB
	if (config.getCommType() == TransformerConfig::AUTO || config.getCommType() == TransformerConfig::RSB) {
		TransformCommRsb::Ptr p(new TransformCommRsb("read-only", allListeners));
		comms.push_back(p);
	}
#endif
#ifdef RCT_HAVE_ROS
	if (config.getCommType() == TransformerConfig::AUTO || config.getCommType() == TransformerConfig::ROS) {
		TransformCommRos::Ptr p(new TransformCommRos(config.getCacheTime(), allListeners));
		comms.push_back();
	}
#endif

#ifndef RCT_HAVE_RSB
#ifndef RCT_HAVE_ROS
	throw TransformerFactoryException("No known communicator implementation available!");
#endif
#endif

	if (comms.empty()) {
		throw TransformerFactoryException(string("Can not generate communicator " + TransformerConfig::typeToString(config.getCommType())));
	}

	//todo
	comms[0]->init(config);
	TransformReceiver::Ptr transformer(new TransformReceiver(core, comms[0], config));
	return transformer;
}

TransformPublisher::Ptr TransformerFactory::createTransformPublisher(const std::string &name, const TransformerConfig& config) const {

	// order is priority
	vector<TransformCommunicator::Ptr> comms;
#ifdef RCT_HAVE_RSB
	if (config.getCommType() == TransformerConfig::AUTO || config.getCommType() == TransformerConfig::RSB) {
		TransformCommRsb::Ptr p(new TransformCommRsb(name));
		comms.push_back(p);
	}
#endif
#ifdef RCT_HAVE_ROS
	if (config.getCommType() == TransformerConfig::AUTO || config.getCommType() == TransformerConfig::ROS) {
		TransformCommRos::Ptr p(new TransformCommRos(config.getCacheTime(), allListeners));
		comms.push_back();
	}
#endif

#ifndef RCT_HAVE_RSB
#ifndef RCT_HAVE_ROS
	throw TransformerFactoryException("No known communicator implementation available!");
#endif
#endif

	if (comms.empty()) {
		throw TransformerFactoryException(string("Can not generate communicator " + TransformerConfig::typeToString(config.getCommType())));
	}

	//todo
	comms[0]->init(config);
	TransformPublisher::Ptr transformer(new TransformPublisher(comms[0], config));
	return transformer;
}
}
