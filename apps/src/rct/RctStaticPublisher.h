/*
 * RctStaticPublisher.h
 *
 *  Created on: Dec 16, 2014
 *      Author: leon
 */

#pragma once
#include <string>
#include <vector>

namespace rct {

class RctStaticPublisher {
public:
	RctStaticPublisher(const std::vector<std::string> &configFiles, bool bridge);
	virtual ~RctStaticPublisher();

	void run();
	void interrupt();
};

} /* namespace rct */
