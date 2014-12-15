/*
 * TransformerRos.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include <rct/impl/TransformerTF2.h>
#include <algorithm>
#include <boost/bind.hpp>
#include "TransformCommRos.h"

using namespace std;

namespace rct {


TransformCommRos::TransformCommRos(
		const boost::posix_time::time_duration& cacheTime): tfListener(tfBuffer) {

	tf2::BufferCore::TransformableCallback cb(boost::bind(&transformCallback, this, _1, _2, _3, _4, _5));
	tf2::TransformableCallbackHandle handle = tfBuffer.addTransformableCallback(cb);
}

TransformCommRos::~TransformCommRos() {
}

bool TransformCommRos::sendTransform(const Transform& transform) {

	geometry_msgs::TransformStamped t;
	TransformerTF2::convertTransformToTf(transform, t);
	return tfBroadcaster.sendTransform(t);
}

bool TransformCommRos::sendTransform(const vector<Transform>& transform) {
	vector<geometry_msgs::TransformStamped> ts;
	vector<Transform>::const_iterator it;
	for (it = transform.begin(); it != transform.end(); ++it) {
		geometry_msgs::TransformStamped t;
		TransformerTF2::convertTransformToTf(*it, t);
		ts.push_back(t);
	}
	return tfBroadcaster.sendTransform(ts);
}

Transform TransformCommRos::lookupTransform(const std::string& target_frame,
		const std::string& source_frame,
		const boost::posix_time::ptime& time) const {

	ros::Time rosTime = ros::Time::fromBoost(time);
	geometry_msgs::TransformStamped t = tfBuffer.lookupTransform(target_frame, source_frame, rosTime);
	Transform tOut;
	TransformerTF2::convertTfToTransform(t, tOut);
	return tOut;
}

Transform TransformCommRos::lookupTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
	ros::Time rosTimeT = ros::Time::fromBoost(target_time);
	ros::Time rosTimeS = ros::Time::fromBoost(source_time);
	geometry_msgs::TransformStamped t = tfBuffer.lookupTransform(target_frame, rosTimeT, source_frame, rosTimeS, fixed_frame);
	Transform tOut;
	TransformerTF2::convertTfToTransform(t, tOut);
	return tOut;
}

bool TransformCommRos::canTransform(const std::string& target_frame,
		const std::string& source_frame, const boost::posix_time::ptime& time,
		std::string* error_msg) const {
	ros::Time rosTime = ros::Time::fromBoost(time);
	return tfBuffer.canTransform(target_frame, source_frame, rosTime);
}

bool TransformCommRos::canTransform(const std::string& target_frame,
		const boost::posix_time::ptime& target_time,
		const std::string& source_frame,
		const boost::posix_time::ptime& source_time,
		const std::string& fixed_frame, std::string* error_msg) const {
	ros::Time rosTimeT = ros::Time::fromBoost(target_time);
	ros::Time rosTimeS = ros::Time::fromBoost(source_time);
	return tfBuffer.canTransform(target_frame, rosTimeT, source_frame, rosTimeS, fixed_frame);
}

void TransformCommRos::addTransformListener(TransformListener::Ptr& listener) {
	listeners.push_back(listener);
}
void TransformCommRos::removeTransformListener(TransformListener::Ptr& listener) {
	vector<TransformListener::Ptr> it = find(listeners.begin(), listeners.end(), listener);
	if (it != listeners.end()) {
		listeners.erase(it);
	}
}

void TransformCommRos::transformCallback(tf2::TransformableRequestHandle handle, const std::string& target, const std::string& source, ros::Time time, tf2::TransformableResult result) {
	if (result == tf2::TransformableResult::TransformAvailable) {
		vector<TransformListener::Ptr> it;
		for (it = listeners.begin(); it != listeners.end(); ++it) {
			TransformListener::Ptr l = *it;
			Transform t = lookupTransform(target, source, time.toBoost());
			l->newTransformAvailable(t);
		}
	}
}

} /* namespace rct */
