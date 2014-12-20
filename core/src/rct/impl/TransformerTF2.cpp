/*
 * TransformerTF2.cpp
 *
 *  Created on: 05.12.2014
 *      Author: leon
 */

#include "../impl/TransformerTF2.h"

using namespace boost;
using namespace std;
using namespace Eigen;

namespace rct {

TransformerTF2::TransformerTF2(const posix_time::time_duration& cacheTime) :
		tfBuffer(ros::Duration().fromNSec(cacheTime.total_nanoseconds())) {
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

bool TransformerTF2::setTransform(const Transform& transform_in,
		const std::string& authority, bool is_static) {

	geometry_msgs::TransformStamped t;
	convertTransformToTf(transform_in, t);

	return tfBuffer.setTransform(t, authority, is_static);
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

void TransformerTF2::newTransformAvailable(const rct::Transform& t) {
	// TODO authority?
	setTransform(t, "");
}

} /* namespace rct */
