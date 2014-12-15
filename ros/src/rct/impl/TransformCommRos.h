/*
 * TransformerRos.h
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#pragma once

#include <rct/impl/TransformCommunicator.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace rct {

class TransformCommRos: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRos> Ptr;
	TransformCommRos(const boost::posix_time::time_duration& cacheTime);
	TransformCommRos(const boost::posix_time::time_duration& cacheTime, TransformListener::Ptr listener);
	virtual ~TransformCommRos();

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
private:
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	std::vector<TransformListener::Ptr> listeners;
	boost::mutex mutex;

	void transformCallback(tf2::TransformableRequestHandle handle, const std::string& target, const std::string& source, ros::Time time, tf2::TransformableResult result);
};

} /* namespace rct */
