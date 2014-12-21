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

class RctStaticPublisher {
public:
	RctStaticPublisher(const std::vector<std::string> &configFiles, bool bridge);
	virtual ~RctStaticPublisher();

	void run();
	void interrupt();

private:
	Transformer::Ptr transformerRsb;
	Transformer::Ptr transformerRos;
	bool bridge;
	bool interrupted;

	boost::condition_variable cond;
	boost::mutex mutex;

	static log4cxx::LoggerPtr logger;
};

} /* namespace rct */
