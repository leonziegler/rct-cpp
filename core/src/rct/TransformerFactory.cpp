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

Transformer::Ptr TransformerFactory::createTransformer(const TransformerConfig& config) const {
	TransformerCore::Ptr core;
#ifdef RCT_HAVE_TF2
	core = TransformerTF2::Ptr(new TransformerTF2(config.getCacheTime()));
#else
	throw TransformerFactoryException("No known logic implementation available!");
#endif

	// order is priority
	vector<TransformCommunicator::Ptr> comms;
#ifdef RCT_HAVE_RSB
	if (config.getCommType() == TransformerConfig::AUTO || config.getCommType() == TransformerConfig::RSB) {
		comms.push_back(TransformCommRsb::Ptr(new TransformCommRsb(config.getCacheTime(), core)));
	}
#endif
#ifdef RCT_HAVE_ROS
	if (config.getCommType() == TransformerConfig::AUTO || config.getCommType() == TransformerConfig::ROS) {
		comms.push_back(TransformCommRos::Ptr(new TransformCommRos(config.getCacheTime(), core)));
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

	comms[0]->init(config);
	Transformer::Ptr transformer(new Transformer(core, comms[0], config));
	return transformer;
}

}
