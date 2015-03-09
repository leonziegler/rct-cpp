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
#include <log4cxx/logger.h>
#include "TransformCommRos.h"
using namespace std;

namespace rct {

log4cxx::LoggerPtr TransformCommRos::logger = log4cxx::Logger::getLogger("rcs.ros.TransformCommRos");

TransformCommRos::TransformCommRos(
		const string &name,
		const boost::posix_time::time_duration& cacheTime):name(name) {
}

TransformCommRos::TransformCommRos(
		const string &name,
		const boost::posix_time::time_duration& cacheTime,
		const TransformListener::Ptr& listener):name(name) {

	addTransformListener(listener);
}

TransformCommRos::TransformCommRos(
		const string &name,
		const boost::posix_time::time_duration& cacheTime,
		const vector<TransformListener::Ptr>& l):name(name) {

	addTransformListener(l);
}

TransformCommRos::~TransformCommRos() {
}

void TransformCommRos::init(const TransformerConfig &conf) {

	LOG4CXX_TRACE(logger, "init()");

	tf2_ros::TransformCallback cb(boost::bind(&TransformCommRos::transformCallback, this, _1, _2, _3, _4, _5));
	tfListener = new tf2_ros::TransformListener(tfBuffer, cb);
}

bool TransformCommRos::sendTransform(const Transform& transform, TransformType type) {

	geometry_msgs::TransformStamped t;
	TransformerTF2::convertTransformToTf(transform, t);
	if (type == STATIC) {
		tfBroadcasterStatic.sendTransform(t);
	} else if (type == DYNAMIC) {
		tfBroadcaster.sendTransform(t);
	} else {
		LOG4CXX_ERROR(logger, "Cannot send transform. Reason: Unknown TransformType: " << type);
		return false;
	}
	return true;
}

bool TransformCommRos::sendTransform(const vector<Transform>& transform, TransformType type) {
	vector<geometry_msgs::TransformStamped> ts;
	vector<Transform>::const_iterator it;
	for (it = transform.begin(); it != transform.end(); ++it) {
		geometry_msgs::TransformStamped t;
		TransformerTF2::convertTransformToTf(*it, t);
		ts.push_back(t);
	}
	if (type == STATIC) {
		tfBroadcasterStatic.sendTransform(ts);
	} else if (type == DYNAMIC) {
		tfBroadcaster.sendTransform(ts);
	} else {
		LOG4CXX_ERROR(logger, "Cannot send transform. Reason: Unknown TransformType: " << type);
		return false;
	}
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

void TransformCommRos::transformCallback(const std::string& target_frame, const std::string& source_frame, ros::Time time, const std::string & authority, bool is_static) {
	LOG4CXX_DEBUG(logger, "Got transform from ROS. tgt:" << target_frame << " src:" << source_frame << " auth:" << authority);
	geometry_msgs::TransformStamped rosTransform = tfBuffer.lookupTransform(target_frame, source_frame, time);
	vector<TransformListener::Ptr>::iterator it;
	boost::mutex::scoped_lock(mutex);
	Transform t;
	TransformerTF2::convertTfToTransform(rosTransform, t);
	t.setAuthority(authority);
	LOG4CXX_DEBUG(logger, "Received transform: " << t);
	for (it = listeners.begin(); it != listeners.end(); ++it) {
		TransformListener::Ptr l = *it;
		l->newTransformAvailable(t, is_static);
	}
	LOG4CXX_TRACE(logger, "Notification done");
}

std::string TransformCommRos::getAuthorityName() const {
	return name;
}

void TransformCommRos::printContents(std::ostream& stream) const {
	stream << "authority = " << name;
	stream << ", communication = ros";
	stream << ", #listeners = " << listeners.size();
}

} /* namespace rct */
