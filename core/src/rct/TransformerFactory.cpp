/*
 * TransformerFactory.cpp
 *
 *  Created on: Dec 7, 2014
 *      Author: leon
 */

#include "TransformerFactory.h"
#include "rctConfig.h"
#ifdef RCT_HAVE_TF2
#include "impl/TransformerTF2.h"
#endif
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

Transformer::Ptr TransformerFactory::createTransformer(const boost::posix_time::time_duration& cacheTime) const {
#ifdef RCT_HAVE_TF2
	return TransformerTF2::Ptr(new TransformerTF2(cacheTime));
#else
	throw TransformerFactoryException("No known logic implementation available!");
#endif
}

}
