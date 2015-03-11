/*
 * TransformerRos.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: leon
 */

#include <rct/impl/TransformerTF2.h>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <tf2/buffer_core.h>
#include <rsc/logging/Logger.h>
#include "TransformCommRos.h"
using namespace std;

namespace rct {

rsc::logging::LoggerPtr TransformCommRos::logger = rsc::logging::Logger::getLogger(
		"rct.ros.TransformCommRos");

TransformCommRos::TransformCommRos(const string &name,
		const boost::posix_time::time_duration& cacheTime, bool legacyMode, long legacyIntervalMSec) :
		name(name), legacyMode(legacyMode), running(true), legacyIntervalMSec(legacyIntervalMSec) {
}

TransformCommRos::TransformCommRos(const string &name,
		const boost::posix_time::time_duration& cacheTime, const TransformListener::Ptr& listener,
		bool legacyMode, long legacyIntervalMSec) :
		name(name), legacyMode(legacyMode), running(true), legacyIntervalMSec(legacyIntervalMSec) {

	addTransformListener(listener);
}

TransformCommRos::TransformCommRos(const string &name,
		const boost::posix_time::time_duration& cacheTime, const vector<TransformListener::Ptr>& l,
		bool legacyMode, long legacyIntervalMSec) :
		name(name), legacyMode(legacyMode), running(true), legacyIntervalMSec(legacyIntervalMSec) {

	addTransformListener(l);
}

TransformCommRos::~TransformCommRos() {
	running = false;
	delete tfListener;
}

void TransformCommRos::init(const TransformerConfig &conf) {

	RSCTRACE(logger, "init()");

	TransformCallbackRos cb(boost::bind(&TransformCommRos::transformCallback, this, _1, _2, _3));
	tfListener = new TransformListenerRos(cb);
}
void TransformCommRos::shutdown() {
	listeners.clear();
	std::map<std::string, boost::thread*>::iterator it;
	for (it = legacyThreadsCache.begin(); it != legacyThreadsCache.end(); ++it) {
		it->second->interrupt();
		it->second->join();
		delete it->second;
	}
	delete tfListener;
}

bool TransformCommRos::sendTransform(const Transform& transform, TransformType type) {

	geometry_msgs::TransformStamped t;
	TransformerTF2::convertTransformToTf(transform, t);
	if (type == STATIC) {
		if (legacyMode) {
			RSCDEBUG(logger, "Send transform on legacy mode broadcaster " << t);
			bool res = sendTransformStaticLegacy(t);
			return res;
		} else {
			RSCDEBUG(logger, "Send transform on static broadcaster " << t);
			tfBroadcasterStatic.sendTransform(t);
			return true;
		}
	} else if (type == DYNAMIC) {
		RSCDEBUG(logger, "Send transform on non-static broadcaster " << t);
		tfBroadcaster.sendTransform(t);
		return true;
	} else {
		RSCERROR(logger, "Cannot send transform. Reason: Unknown TransformType: " << type);
		return false;
	}
	return true;
}

bool TransformCommRos::sendTransformStaticLegacy(const geometry_msgs::TransformStamped& transform) {

	string cacheKey = transform.header.frame_id + transform.child_frame_id;

	ros::Duration interval(float(legacyIntervalMSec) / 1000.0);
	if (legacyThreadsCache.count(cacheKey)) {
		legacyThreadsCache[cacheKey]->interrupt();
		legacyThreadsCache[cacheKey]->timed_join(boost::posix_time::seconds(2));
		delete legacyThreadsCache[cacheKey];
	}

	legacyThreadsCache[cacheKey] = new boost::thread(
			boost::bind(&TransformCommRos::transformLegacyPublish, this, transform, interval));

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
		RSCDEBUG(logger, "Send transform on static broadcaster " << ts);
		tfBroadcasterStatic.sendTransform(ts);
	} else if (type == DYNAMIC) {
		RSCDEBUG(logger, "Send transform on non-static broadcaster " << ts);
		tfBroadcaster.sendTransform(ts);
	} else {
		RSCERROR(logger, "Cannot send transform. Reason: Unknown TransformType: " << type);
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
	vector<TransformListener::Ptr>::iterator it = find(listeners.begin(), listeners.end(),
			listener);
	if (it != listeners.end()) {
		listeners.erase(it);
	}
}

void TransformCommRos::transformCallback(const geometry_msgs::TransformStamped rosTransform,
		const std::string & authority, bool is_static) {

	if (authority == ros::this_node::getName()) {
		RSCTRACE(logger,
				"Received transform from myself. Ignore. (authority: " << authority << ")");
		return;
	}

	string authorityClean = authority;
	boost::algorithm::replace_all(authorityClean, "/", "");

	RSCTRACE(logger,
			"Got transform from ROS. parent:" << rosTransform.header.frame_id << " child:" << rosTransform.child_frame_id << " auth:" << authorityClean);

	vector<TransformListener::Ptr>::iterator it;
	boost::mutex::scoped_lock(mutex);
	Transform t;
	TransformerTF2::convertTfToTransform(rosTransform, t);
	t.setAuthority(authorityClean);
	RSCDEBUG(logger, "Received transform: " << t);
	for (it = listeners.begin(); it != listeners.end(); ++it) {
		TransformListener::Ptr l = *it;
		l->newTransformAvailable(t, is_static);
	}
	RSCTRACE(logger, "Notification done");
}

void TransformCommRos::transformLegacyPublish(geometry_msgs::TransformStamped t,
		ros::Duration sleeper) {
	while (running && !boost::this_thread::interruption_requested()) {
		try {
			t.header.stamp = ros::Time::now() + sleeper;
			tfBroadcaster.sendTransform(t);
			sleeper.sleep();
		} catch (std::exception &e) {
			RSCERROR(logger, "Cannot send transform. Reason: " << e.what());
			usleep(100 * 1000);
		}
	}
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
