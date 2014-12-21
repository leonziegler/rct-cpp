/*
 * RctStaticPublisher.h
 *
 *  Created on: Dec 16, 2014
 *      Author: leon
 */

#pragma once
#include <string>
#include <vector>
#include <rct/TransformerFactory.h>
#include <boost/thread.hpp>
#include <log4cxx/logger.h>

namespace rct {

class RctStaticPublisher;

class Handler: public TransformListener {
public:
	typedef boost::shared_ptr<Handler> Ptr;
	Handler(RctStaticPublisher* parent): parent(parent) {
	}
	virtual ~Handler() {
	}
	void newTransformAvailable(const Transform& transform);
	bool hasTransforms();
	Transform nextTransform();
private:
	RctStaticPublisher* parent;
	boost::mutex mutex;
	std::vector<Transform> transforms;
};

class RctStaticPublisher {
public:
	RctStaticPublisher(const std::vector<std::string> &configFiles, bool bridge);
	virtual ~RctStaticPublisher();

	void run();
	void interrupt();
	void notify();

private:
	Transformer::Ptr transformerRsb;
	TransformCommunicator::Ptr commRos;
	Handler::Ptr rosHandler;
	Handler::Ptr rsbHandler;
	bool bridge;
	bool interrupted;

	boost::condition_variable cond;
	boost::mutex mutex;

	static log4cxx::LoggerPtr logger;
};

} /* namespace rct */
