/*
 * TransformerRos.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include <rct/impl/TransformerTF2.h>
#include <algorithm>
#include <boost/bind.hpp>
#include <tf2/buffer_core.h>
#include "TransformCommRos.h"

using namespace std;

namespace rct {

TransformCommRos::TransformCommRos(
		const boost::posix_time::time_duration& cacheTime) {
}

TransformCommRos::TransformCommRos(
		const boost::posix_time::time_duration& cacheTime, const TransformListener::Ptr& listener) {

	addTransformListener(listener);
}

TransformCommRos::TransformCommRos(
		const boost::posix_time::time_duration& cacheTime,
		const vector<TransformListener::Ptr>& l) {

	addTransformListener(l);
}

TransformCommRos::~TransformCommRos() {
}

void TransformCommRos::init(const TransformerConfig &conf) {
	tf2::BufferCore::TransformableCallback cb(boost::bind(&TransformCommRos::transformCallback, this, _1, _2, _3, _4, _5));
	tf2::TransformableCallbackHandle handle = tfBuffer.addTransformableCallback(cb);

	tfListener = new tf2_ros::TransformListener(tfBuffer);
}

bool TransformCommRos::sendTransform(const Transform& transform) {

	geometry_msgs::TransformStamped t;
	TransformerTF2::convertTransformToTf(transform, t);
	tfBroadcaster.sendTransform(t);
	return true;
}

bool TransformCommRos::sendTransform(const vector<Transform>& transform) {
	vector<geometry_msgs::TransformStamped> ts;
	vector<Transform>::const_iterator it;
	for (it = transform.begin(); it != transform.end(); ++it) {
		geometry_msgs::TransformStamped t;
		TransformerTF2::convertTransformToTf(*it, t);
		ts.push_back(t);
	}
	tfBroadcaster.sendTransform(ts);
	return true;
}

void TransformCommRos::addTransformListener(const TransformListener::Ptr& listener) {
	boost::mutex::scoped_lock(mutex);
	listeners.push_back(listener);
}

void TransformCommRos::addTransformListener(const vector<TransformListener::Ptr>& l) {
	boost::mutex::scoped_lock(mutex);
	listeners.insert(listeners.end(), l.begin(), l.end());
}
void TransformCommRos::removeTransformListener(const TransformListener::Ptr& listener) {
	boost::mutex::scoped_lock(mutex);
	vector<TransformListener::Ptr>::iterator it = find(listeners.begin(), listeners.end(), listener);
	if (it != listeners.end()) {
		listeners.erase(it);
	}
}

void TransformCommRos::transformCallback(tf2::TransformableRequestHandle handle, const std::string& target, const std::string& source, ros::Time time, tf2::TransformableResult result) {
	if (result == tf2::TransformAvailable) {
		geometry_msgs::TransformStamped rosTransform = tfBuffer.lookupTransform(target, source, time);
		vector<TransformListener::Ptr>::iterator it;
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
