/*
 * Transformer.h
 *
 *  Created on: 05.12.2014
 *      Author: leon
 */

#pragma once

#include "../Transform.h"
#include "TransformListener.h"
#include <Eigen/Geometry>
#include <string>
#include <boost/integer.hpp>
#include <boost/thread.hpp>

namespace rct {

class TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommunicator> Ptr;
	TransformCommunicator();
	virtual ~TransformCommunicator();


	/** \brief Add transform information to the rct data structure
	 * \param transform The transform to store
	 * \param authority The source of the information for this transform
	 * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
	 * \return True unless an error occured
	 */
	virtual bool sendTransform(const Transform& transform) = 0;

	/** \brief Add transform information to the rct data structure
	 * \param transform The transform to store
	 * \param authority The source of the information for this transform
	 * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
	 * \return True unless an error occured
	 */
	virtual bool sendTransform(const std::vector<Transform>& transforms) = 0;

	virtual void addTransformListener(TransformListener::Ptr& listener) = 0;
	virtual void removeTransformListener(TransformListener::Ptr& listener) = 0;
};

} /* namespace rct */
