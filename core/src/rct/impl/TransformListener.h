/*
 * TransformListener.h
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#pragma once

#include <rct/Transform.h>
#include <boost/shared_ptr.hpp>

namespace rct {

class TransformListener {
public:
	typedef boost::shared_ptr<TransformListener> Ptr;
	TransformListener() {
	}
	virtual ~TransformListener() {
	}

	virtual void newTransformAvailable(const Transform& transform) = 0;

};

}  // namespace rct
