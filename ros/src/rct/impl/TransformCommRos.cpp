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
		const boost::posix_time::time_duration& cacheTime) {

	tf2::BufferCore::TransformableCallback cb(boost::bind(&transformCallback, this, _1, _2, _3, _4, _5));
	tf2::TransformableCallbackHandle handle = tfBuffer.addTransformableCallback(cb);

	tfListener = tf2_ros::TransformListener(tfBuffer);
}

TransformCommRos::TransformCommRos(
		const boost::posix_time::time_duration& cacheTime, TransformListener::Ptr listener) {

	addTransformListener(listener);

	tf2::BufferCore::TransformableCallback cb(boost::bind(&transformCallback, this, _1, _2, _3, _4, _5));
	tf2::TransformableCallbackHandle handle = tfBuffer.addTransformableCallback(cb);

	tfListener = tf2_ros::TransformListener(tfBuffer);
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

void TransformCommRos::addTransformListener(TransformListener::Ptr& listener) {
	boost::mutex::scoped_lock(mutex);
	listeners.push_back(listener);
}
void TransformCommRos::removeTransformListener(TransformListener::Ptr& listener) {
	boost::mutex::scoped_lock(mutex);
	vector<TransformListener::Ptr> it = find(listeners.begin(), listeners.end(), listener);
	if (it != listeners.end()) {
		listeners.erase(it);
	}
}

void TransformCommRos::transformCallback(tf2::TransformableRequestHandle handle, const std::string& target, const std::string& source, ros::Time time, tf2::TransformableResult result) {
	if (result == tf2::TransformableResult::TransformAvailable) {
		geometry_msgs::TransformStamped rosTransform = tfBuffer.lookupTransform(target, source, time);
		vector<TransformListener::Ptr> it;
		boost::mutex::scoped_lock(mutex);
		for (it = listeners.begin(); it != listeners.end(); ++it) {
			TransformListener::Ptr l = *it;
			Transform t;
			TransformerTF2::convertTfToTransform(rosTransform, t);
			l->newTransformAvailable(t);
		}
	}
}

} /* namespace rct */
