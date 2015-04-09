/*
 * TransformerTF2.cpp
 *
 *  Created on: 05.12.2014
 *      Author: leon
 */

#include "../impl/TransformerTF2.h"

#include <boost/algorithm/string.hpp>

#include <tf2/exceptions.h>

using namespace boost;
using namespace std;
using namespace Eigen;
using namespace tf2;

namespace rct {

rsc::logging::LoggerPtr TransformerTF2::logger = rsc::logging::Logger::getLogger("rct.core.TransformerTF2");

TransformerTF2::TransformerTF2(const posix_time::time_duration& cacheTime) :
		tfBuffer(ros::Duration().fromNSec(cacheTime.total_nanoseconds())) {

	tfBuffer._addTransformsChangedListener(bind(&TransformerTF2::tfChanged, this));
}

TransformerTF2::~TransformerTF2() {
}

void TransformerTF2::convertTransformToTf(const Transform &t,
		geometry_msgs::TransformStamped &tOut) {

	Quaterniond eigenQuat = t.getRotationQuat();
	tOut.header.frame_id = t.getFrameParent();
	tOut.header.stamp = ros::Time::fromBoost(t.getTime());
	tOut.child_frame_id = t.getFrameChild();
	tOut.transform.rotation.w = eigenQuat.w();
	tOut.transform.rotation.x = eigenQuat.x();
	tOut.transform.rotation.y = eigenQuat.y();
	tOut.transform.rotation.z = eigenQuat.z();
	tOut.transform.translation.x = t.getTranslation()(0);
	tOut.transform.translation.y = t.getTranslation()(1);
	tOut.transform.translation.z = t.getTranslation()(2);
}
void TransformerTF2::convertTfToTransform(const geometry_msgs::TransformStamped &t,
		Transform &tOut) {
	tOut.setFrameChild(t.child_frame_id);
	tOut.setFrameParent(t.header.frame_id);
	tOut.setTime(t.header.stamp.toBoost());

	Eigen::Vector3d p(t.transform.translation.x, t.transform.translation.y,
			t.transform.translation.z);
	Eigen::Quaterniond r(t.transform.rotation.w, t.transform.rotation.x,
			t.transform.rotation.y,	t.transform.rotation.z);
	Eigen::Affine3d a = Eigen::Affine3d().fromPositionOrientationScale(p, r, Vector3d::Ones());
	tOut.setTransform(a);
}

void TransformerTF2::clear() {
	tfBuffer.clear();
}

bool TransformerTF2::setTransform(const Transform& transform_in, bool is_static) {

	geometry_msgs::TransformStamped t;
	convertTransformToTf(transform_in, t);

	return tfBuffer.setTransform(t, transform_in.getAuthority(), is_static);
}

Transform TransformerTF2::lookupTransform(const std::string& target_frame,
		const std::string& source_frame, const posix_time::ptime& time) const {

	geometry_msgs::TransformStamped t0 = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time().fromBoost(time));
	Transform t1;
	convertTfToTransform(t0, t1);
	return t1;
}

Transform TransformerTF2::lookupTransform(const std::string& target_frame,
		const posix_time::ptime& target_time, const std::string& source_frame,
		const posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
	geometry_msgs::TransformStamped t0 = tfBuffer.lookupTransform(target_frame, ros::Time().fromBoost(target_time), source_frame, ros::Time().fromBoost(source_time), fixed_frame);
	Transform t1;
	convertTfToTransform(t0, t1);
	return t1;
}

TransformerTF2::FuturePtr TransformerTF2::requestTransform(
		const std::string& target_frame, const std::string& source_frame,
		const boost::posix_time::ptime& time) {

	FuturePtr result(new FutureType());

	boost::mutex::scoped_lock lock(inprogressMutex);
	try {
		Transform t = lookupTransform(target_frame, source_frame, time);
		result->set(t);
		RSCTRACE(this->logger, "Lookup possible before request applies. Take shortcut.");

	} catch (tf2::LookupException &e) {

		RSCTRACE(this->logger, "Lookup NOT possible before request applies. Register request.");

		this->requestsInProgress.insert(
				std::make_pair(Request(target_frame, source_frame, ros::Time().fromBoost(time)),
						result));
	} catch (tf2::ExtrapolationException &e) {

		RSCTRACE(this->logger, "Lookup NOT possible before request applies. Register request.");

		this->requestsInProgress.insert(
				std::make_pair(Request(target_frame, source_frame, ros::Time().fromBoost(time)),
						result));
	}

	return result;
}

void TransformerTF2::tfChanged() {
	boost::mutex::scoped_lock lock(inprogressMutex);

	if (firstChangeTime.isZero()) {
		boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
		firstChangeTime = ros::Time().fromBoost(now);
	}

	std::map<Request, FuturePtr>::iterator it;
	for (it = requestsInProgress.begin(); it != requestsInProgress.end(); ++it) {
		try {
			ros::Time t = it->first.time;
			if (t < firstChangeTime) {
				t = firstChangeTime;
			}
			geometry_msgs::TransformStamped t0 = tfBuffer.lookupTransform(it->first.target_frame,
					it->first.source_frame, t);
			Transform t1;
			convertTfToTransform(t0, t1);
			it->second->set(t1);
			requestsInProgress.erase(it);
		} catch (tf2::LookupException &e) {
			RSCTRACE(this->logger, "Not yet transformable ");
		} catch (tf2::ExtrapolationException &e) {
			RSCTRACE(this->logger, "Not yet transformable ");
		}
	}
}

bool TransformerTF2::canTransform(const std::string& target_frame, const std::string& source_frame,
		const posix_time::ptime& time, std::string* error_msg) const {
	return tfBuffer.canTransform(target_frame, source_frame, ros::Time().fromBoost(time), error_msg);
}

bool TransformerTF2::canTransform(const std::string& target_frame,
		const posix_time::ptime& target_time, const std::string& source_frame,
		const posix_time::ptime& source_time, const std::string& fixed_frame,
		std::string* error_msg) const {
	return tfBuffer.canTransform(target_frame, ros::Time().fromBoost(target_time), source_frame,
			ros::Time().fromBoost(source_time), fixed_frame, error_msg);
}

void TransformerTF2::newTransformAvailable(const rct::Transform& t, bool isStatic) {
	setTransform(t, isStatic);
}

void TransformerTF2::printContents(std::ostream& stream) const {
	stream << "backend = tf2::BufferCore";
}

std::vector<std::string> TransformerTF2::getFrameStrings() const {
	std::vector<std::string> frames;
	tfBuffer._getFrameStrings(frames);
	return frames;
}

bool TransformerTF2::frameExists(const std::string& frame_id_str) const {
	return tfBuffer._frameExists(frame_id_str);
}

std::string TransformerTF2::getParent(const std::string& frame_id,
		const boost::posix_time::ptime &time) const {
	string p;
	tfBuffer._getParent(frame_id, ros::Time().fromBoost(time), p);
	return p;
}

std::string TransformerTF2::allFramesAsDot() const {
	std::string dot = tfBuffer._allFramesAsDot();
	boost::algorithm::replace_all(dot, " tf ", " RCT ");
	boost::algorithm::replace_all(dot, "Broadcaster:", "Publisher:");
	boost::algorithm::replace_all(dot,
			"Average rate: 10000.000 Hz\\nMost recent transform: 0.000 \\nBuffer length: 0.000 sec",
			"Static transform");
	boost::algorithm::replace_all(dot, "Most recent transform", "Most recent");
	return dot;
}

std::string TransformerTF2::allFramesAsYAML() const {
	return tfBuffer.allFramesAsYAML();
}

std::string TransformerTF2::allFramesAsString() const {
	return tfBuffer.allFramesAsString();
}

} /* namespace rct */

