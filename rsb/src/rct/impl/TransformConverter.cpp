/*
 * TransformConverter.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: lziegler
 */

#include "TransformConverter.h"
#include <rct/Transform.h>

#include <rsb/converter/SerializationException.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <rct/FrameTransform.pb.h>
#include <rst/geometry/Pose.pb.h>

using namespace std;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;

namespace rct {

void domainToRST(const Transform& transform, FrameTransform &t) {
	boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
	boost::posix_time::time_duration::tick_type microTime =
			(transform.getTime() - epoch).total_microseconds();

	t.set_frame_parent(transform.getFrameParent());
	t.set_frame_child(transform.getFrameChild());
	t.mutable_time()->set_time(microTime);
	t.mutable_transform()->mutable_translation()->set_x(transform.getTranslation().x());
	t.mutable_transform()->mutable_translation()->set_y(transform.getTranslation().y());
	t.mutable_transform()->mutable_translation()->set_z(transform.getTranslation().z());
	t.mutable_transform()->mutable_rotation()->set_qw(transform.getRotationQuat().w());
	t.mutable_transform()->mutable_rotation()->set_qx(transform.getRotationQuat().x());
	t.mutable_transform()->mutable_rotation()->set_qy(transform.getRotationQuat().y());
	t.mutable_transform()->mutable_rotation()->set_qz(transform.getRotationQuat().z());
}
void rstToDomain(const FrameTransform &t, Transform& transform) {
	const boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
	const boost::posix_time::ptime time = epoch + boost::posix_time::microseconds(t.time().time());

	Eigen::Vector3d p(t.transform().translation().x(), t.transform().translation().y(),
			t.transform().translation().z());
	Eigen::Quaterniond r(t.transform().rotation().qw(), t.transform().rotation().qx(),
			t.transform().rotation().qy(), t.transform().rotation().qz());
	Eigen::Affine3d a = Eigen::Affine3d().fromPositionOrientationScale(p, r,
			Eigen::Vector3d::Ones());

	transform.setFrameParent(t.frame_parent());
	transform.setFrameChild(t.frame_child());
	transform.setTime(time);
	transform.setTransform(a);
}

TransformConverter::TransformConverter():
                rsb::converter::Converter<string>(
                        rsc::runtime::typeName<FrameTransform>(),
                        RSB_TYPE_TAG(Transform)) {
	converter = boost::shared_ptr<Converter<string> >(new ProtocolBufferConverter<FrameTransform>);
}

TransformConverter::~TransformConverter() {
}

std::string TransformConverter::getWireSchema() const {
	return converter->getWireSchema();
}

std::string TransformConverter::serialize(const rsb::AnnotatedData& data, std::string& wire) {
	// Cast to original domain type
    boost::shared_ptr<Transform> domain = static_pointer_cast<Transform>(data.second);

	// Fill protocol buffer object
	rct::FrameTransform proto;

	//
	domainToRST(*domain, proto);

	// Use embedded ProtoBuf converter for serialization to wire
	return converter->serialize(
			make_pair(rsc::runtime::typeName<rct::FrameTransform>(),
					boost::shared_ptr<void>(&proto, rsc::misc::NullDeleter())), wire);
}

rsb::AnnotatedData TransformConverter::deserialize(const std::string& wireType,
		const std::string& wire) {

	// Deserialize and cast to specific ProtoBuf type
	boost::shared_ptr<rct::FrameTransform> proto = boost::static_pointer_cast<rct::FrameTransform>(
			converter->deserialize(wireType, wire).second);

	// Instantiate domain object
	boost::shared_ptr<Transform> domain(new Transform());

	// Read domain data from ProtoBuf
	rstToDomain(*proto, *domain);

	return rsb::AnnotatedData(getDataType(), domain);
}

} /* namespace rct */
