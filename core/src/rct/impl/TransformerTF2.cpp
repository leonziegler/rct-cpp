/*
 * TransformerTF2.cpp
 *
 *  Created on: 05.12.2014
 *      Author: leon
 */

#include "../impl/TransformerTF2.h"

#include <boost/algorithm/string.hpp>

using namespace boost;
using namespace std;
using namespace Eigen;
using namespace tf2;

namespace rct {

log4cxx::LoggerPtr TransformerTF2::logger = log4cxx::Logger::getLogger("rct.core.TransformerTF2");

TransformerTF2::TransformerTF2(const posix_time::time_duration& cacheTime) :
		tfBuffer(ros::Duration().fromNSec(cacheTime.total_nanoseconds())) {

	tf2::BufferCore::TransformableCallback f1(bind(&TransformerTF2::tfRequestCallback, this, _1, _2, _3, _4, _5));
	tfCallbackHandle = tfBuffer.addTransformableCallback(f1);
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
			t.transform.rotation.y, t.transform.rotation.z);
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
	TransformableRequestHandle handle = tfBuffer.addTransformableRequest(tfCallbackHandle, target_frame, source_frame, ros::Time().fromBoost(time));
	{
		boost::mutex::scoped_lock lock(inprogressMutex);
		this->inprogress.insert(std::make_pair(handle, result));
	}
	return result;
}

void TransformerTF2::tfRequestCallback(TransformableRequestHandle request_handle, const std::string& target_frame, const std::string& source_frame,
                                   ros::Time time, TransformableResult tresult) {
	FuturePtr result;
    {
        boost::mutex::scoped_lock lock(this->inprogressMutex);
        map<TransformableRequestHandle, FuturePtr>::const_iterator it
            = this->inprogress.find(request_handle);
        if (it != this->inprogress.end()) {
            result = it->second;
            this->inprogress.erase(request_handle);
        }
    }
    if (!result) {
        LOG4CXX_DEBUG(this->logger, "Received uninteresting callback " << target_frame << " -> " << source_frame);
        return;
    }
    LOG4CXX_DEBUG(this->logger, "Received reply callback " << target_frame << " -> " << source_frame);
    if (tresult == TransformFailure) {
        result->setError("Transform Failure");
    } else {
    	geometry_msgs::TransformStamped t0 = tfBuffer.lookupTransform(target_frame, source_frame, time);
		Transform t1;
		convertTfToTransform(t0, t1);
        result->set(t1);
    }
}

bool TransformerTF2::canTransform(const std::string& target_frame,
		const std::string& source_frame, const posix_time::ptime& time,
		std::string* error_msg) const {
	return tfBuffer.canTransform(target_frame, source_frame, ros::Time().fromBoost(time), error_msg);
}

bool TransformerTF2::canTransform(const std::string& target_frame,
		const posix_time::ptime& target_time, const std::string& source_frame,
		const posix_time::ptime& source_time, const std::string& fixed_frame,
		std::string* error_msg) const {
	return tfBuffer.canTransform(target_frame, ros::Time().fromBoost(target_time), source_frame, ros::Time().fromBoost(source_time), fixed_frame, error_msg);
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

std::string TransformerTF2::getParent(const std::string& frame_id, const boost::posix_time::ptime &time) const {
	string p;
	tfBuffer._getParent(frame_id, ros::Time().fromBoost(time), p);
	return p;
}

std::string TransformerTF2::allFramesAsDot() const {
	std::string dot = tfBuffer._allFramesAsDot();
	boost::algorithm::replace_all(dot, " tf ", " RCT ");
	boost::algorithm::replace_all(dot, "Broadcaster:", "Publisher:");
	boost::algorithm::replace_all(dot, "Average rate: 10000.000 Hz\\nMost recent transform: 0.000 \\nBuffer length: 0.000 sec", "Static");
	return dot;
}

std::string TransformerTF2::allFramesAsYAML() const {
	return tfBuffer.allFramesAsYAML();
}

std::string TransformerTF2::allFramesAsString() const {
	return tfBuffer.allFramesAsString();
}

} /* namespace rct */

