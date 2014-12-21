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
#include <tf2_ros/static_transform_broadcaster.h>

namespace rct {

class TransformCommRos: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRos> Ptr;
	TransformCommRos(const boost::posix_time::time_duration& cacheTime);
	TransformCommRos(const boost::posix_time::time_duration& cacheTime, const TransformListener::Ptr& listener);
	TransformCommRos(const boost::posix_time::time_duration& cacheTime, const std::vector<TransformListener::Ptr>& listener);
	virtual ~TransformCommRos();

	virtual void init(const TransformerConfig &conf);

	virtual bool sendTransform(const Transform& transform, bool isStatic);
	virtual bool sendTransform(const std::vector<Transform>& transforms, bool isStatic);

	virtual void addTransformListener(const TransformListener::Ptr& listener);
	virtual void addTransformListener(const std::vector<TransformListener::Ptr>& listeners);
	virtual void removeTransformListener(const TransformListener::Ptr& listener);

	void printContents(std::ostream& stream) const;
	virtual std::string getAuthorityName() const;
private:
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener* tfListener;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	tf2_ros::StaticTransformBroadcaster tfBroadcasterStatic;
	std::vector<TransformListener::Ptr> listeners;
	boost::mutex mutex;

	void transformCallback(tf2::TransformableRequestHandle handle, const std::string& target, const std::string& source, ros::Time time, tf2::TransformableResult result);
};

} /* namespace rct */
