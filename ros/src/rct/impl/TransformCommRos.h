/*
 * TransformerRos.h
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#pragma once

#include <rct/impl/TransformCommunicator.h>
#include <rct/rctConfig.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "TransformListenerRos.h"

namespace rct {

class TransformCommRos: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRos> Ptr;
	TransformCommRos(const std::string &name, const boost::posix_time::time_duration& cacheTime, bool legacyMode = false, long legacyIntervalMSec = 100);
	TransformCommRos(const std::string &name, const boost::posix_time::time_duration& cacheTime, const TransformListener::Ptr& listener, bool legacyMode = false, long legacyIntervalMSec = 100);
	TransformCommRos(const std::string &name, const boost::posix_time::time_duration& cacheTime, const std::vector<TransformListener::Ptr>& listener, bool legacyMode = false, long legacyIntervalMSec = 100);
	virtual ~TransformCommRos();

	virtual void init(const TransformerConfig &conf);
	virtual void shutdown();

	virtual bool sendTransform(const Transform& transform, TransformType type);
	virtual bool sendTransform(const std::vector<Transform>& transforms, TransformType type);

	virtual void addTransformListener(const TransformListener::Ptr& listener);
	virtual void addTransformListener(const std::vector<TransformListener::Ptr>& listeners);
	virtual void removeTransformListener(const TransformListener::Ptr& listener);

	void printContents(std::ostream& stream) const;
	virtual std::string getAuthorityName() const;
private:

	TransformListenerRos* tfListener;
	tf2_ros::TransformBroadcaster tfBroadcaster;
	tf2_ros::StaticTransformBroadcaster tfBroadcasterStatic;

	std::vector<TransformListener::Ptr> listeners;
	boost::mutex mutex;
	std::string name;

	bool running;
	bool legacyMode;
	long legacyIntervalMSec;
	std::map<std::string, boost::thread*> legacyThreadsCache;

	static rsc::logging::LoggerPtr logger;
	bool sendTransformStaticLegacy(const geometry_msgs::TransformStamped& transform);
	void transformCallback(const geometry_msgs::TransformStamped transform, const std::string & authority, bool is_static);
	void transformLegacyPublish(geometry_msgs::TransformStamped t, ros::Duration sleeper);
};

} /* namespace rct */
