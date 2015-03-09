/*
 * TransformerRos.h
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#pragma once

#include <rct/impl/TransformCommunicator.h>

#include <tf2_ros/buffer.h>
#include "transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace rct {

class TransformCommRos: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRos> Ptr;
	TransformCommRos(const std::string &name, const boost::posix_time::time_duration& cacheTime);
	TransformCommRos(const std::string &name, const boost::posix_time::time_duration& cacheTime, const TransformListener::Ptr& listener);
	TransformCommRos(const std::string &name, const boost::posix_time::time_duration& cacheTime, const std::vector<TransformListener::Ptr>& listener);
	virtual ~TransformCommRos();

	virtual void init(const TransformerConfig &conf);

	virtual bool sendTransform(const Transform& transform, TransformType type);
	virtual bool sendTransform(const std::vector<Transform>& transforms, TransformType type);

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
	std::string name;

	static log4cxx::LoggerPtr logger;

	void transformCallback(const std::string& target_frame, const std::string& source_frame, ros::Time time, const std::string & authority, bool is_static);
};

} /* namespace rct */
