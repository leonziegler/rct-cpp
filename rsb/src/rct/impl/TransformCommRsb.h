/*
 * TransformCommRsb.h
 *
 *  Created on: Dec 15, 2014
 *      Author: leon
 */

#pragma once

#include <rct/impl/TransformCommunicator.h>
#include <rsb/Listener.h>
#include <rsb/Informer.h>

namespace rct {

class TransformCommRsb: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRos> Ptr;
	TransformCommRsb(const boost::posix_time::time_duration& cacheTime);
	virtual ~TransformCommRsb();

private:
	rsb::
};
}  // namespace rct
