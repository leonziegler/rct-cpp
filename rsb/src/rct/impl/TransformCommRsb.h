/*
 * TransformCommRsb.h
 *
 *  Created on: Dec 15, 2014
 *      Author: leon
 */

#pragma once

#include <rct/impl/TransformCommunicator.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <rsb/Listener.h>
#include <rsb/Informer.h>
#include <boost/shared_ptr.hpp>

#include "FrameTransform.pb.h"

namespace rct {

class TransformCommRsb: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRsb> Ptr;
	TransformCommRsb(const boost::posix_time::time_duration& cacheTime, TransformListener::Ptr& listener);
	virtual ~TransformCommRsb();

	/** \brief Add transform information to the rct data structure
	 * \param transform The transform to store
	 * \param authority The source of the information for this transform
	 * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
	 * \return True unless an error occured
	 */
	virtual bool sendTransform(const Transform& transform);
	virtual bool sendTransform(const std::vector<Transform>& transforms);

	virtual void addTransformListener(TransformListener::Ptr& listener);
	virtual void removeTransformListener(TransformListener::Ptr& listener);

	static void convertTransformToPb(const Transform& transform, boost::shared_ptr<FrameTransform> &t);
	static void convertPbToTransform(const boost::shared_ptr<FrameTransform> &t, Transform& transform);

private:
	rsb::ListenerPtr listener;
	rsb::Informer<FrameTransform>::Ptr informer;
	std::vector<TransformListener::Ptr> listeners;
	boost::mutex mutex;

	void frameTransformCallback(const boost::shared_ptr<FrameTransform> &t);
};
}  // namespace rct
