/*
 * TransformCommRsb.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: leon
 */

#include "TransformCommRsb.h"
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

using namespace std;
using namespace rsb;
using namespace boost;

namespace rct {

TransformCommRsb::TransformCommRsb(
		const boost::posix_time::time_duration& cacheTime,
		const TransformListener::Ptr& l) {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(
		const boost::posix_time::time_duration& cacheTime,
		const vector<TransformListener::Ptr>& l) {

	addTransformListener(l);
}

TransformCommRsb::~TransformCommRsb() {
}

void TransformCommRsb::init(const TransformerConfig &conf) {
	converter::ProtocolBufferConverter<FrameTransform>::Ptr converter0(
			new rsb::converter::ProtocolBufferConverter<FrameTransform>());
	converter::converterRepository<string>()->registerConverter(converter0);

	Factory &factory = rsb::getFactory();

	rsbListenerTransform = factory.createListener("/rct/transform");
	rsbInformerTransform = factory.createInformer<FrameTransform>("/rct/transform");
	rsbListenerTrigger = factory.createListener("/rct/trigger");
	rsbInformerTrigger = factory.createInformer<void>("/rct/trigger");

	DataFunctionHandler<FrameTransform>::DataFunction f0(
			bind(&TransformCommRsb::frameTransformCallback, this, _1));
	rsbListenerTransform->addHandler(
			HandlerPtr(new DataFunctionHandler<FrameTransform>(f0)));

	DataFunctionHandler<void>::DataFunction f1(
			bind(&TransformCommRsb::triggerCallback, this, _1));
	rsbListenerTrigger->addHandler(
			HandlerPtr(new DataFunctionHandler<void>(f1)));

	requestSync();

}
void TransformCommRsb::requestSync() {

	if (!rsbInformerTrigger) {
		throw std::runtime_error("communicator was not initialized!");
	}

	// trigger other instances to send transforms
	rsbInformerTrigger->publish(shared_ptr<void>());
}

bool TransformCommRsb::sendTransform(const Transform& transform) {
	if (!rsbInformerTransform) {
		throw std::runtime_error("communicator was not initialized!");
	}

	boost::shared_ptr<FrameTransform> t(new FrameTransform());
	convertTransformToPb(transform, t);
	string cacheKey = transform.getFrameParent() + transform.getFrameChild();

	boost::mutex::scoped_lock(mutex);
	sendCache[cacheKey] = t;
	rsbInformerTransform->publish(t);
	return true;
}

bool TransformCommRsb::sendTransform(const std::vector<Transform>& transforms) {
	std::vector<Transform>::const_iterator it;
	for (it = transforms.begin(); it != transforms.end(); ++it) {
		sendTransform(*it);
	}
	return true;
}

void TransformCommRsb::publishCache() {
	map<string, shared_ptr<FrameTransform> >::iterator it;
	for (it = sendCache.begin(); it != sendCache.end(); it++) {
		rsbInformerTransform->publish(it->second);
	}
}

void TransformCommRsb::addTransformListener(const TransformListener::Ptr& l) {
	boost::mutex::scoped_lock(mutex);
	listeners.push_back(l);
}

void TransformCommRsb::addTransformListener(const vector<TransformListener::Ptr>& l) {
	boost::mutex::scoped_lock(mutex);
	listeners.insert(listeners.end(), l.begin(), l.end());
}

void TransformCommRsb::removeTransformListener(
		const TransformListener::Ptr& l) {
	boost::mutex::scoped_lock(mutex);
	vector<TransformListener::Ptr>::iterator it = find(listeners.begin(),
			listeners.end(), l);
	if (it != listeners.end()) {
		listeners.erase(it);
	}
}

void TransformCommRsb::frameTransformCallback(
		const boost::shared_ptr<FrameTransform> &t) {
	vector<TransformListener::Ptr>::iterator it;
	boost::mutex::scoped_lock(mutex);
	for (it = listeners.begin(); it != listeners.end(); ++it) {
		TransformListener::Ptr l = *it;
		Transform transform;
		convertPbToTransform(t, transform);
		l->newTransformAvailable(transform);
	}
}

void TransformCommRsb::triggerCallback(const shared_ptr<void> &t) {
	// publish send cache as thread
	boost::thread workerThread(&TransformCommRsb::publishCache, this);
}

void TransformCommRsb::convertTransformToPb(const Transform& transform,
		boost::shared_ptr<FrameTransform> &t) {

	boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
	boost::posix_time::time_duration::tick_type microTime = (transform.getTime()
			- epoch).total_microseconds();

	t->set_frame_parent(transform.getFrameParent());
	t->set_frame_child(transform.getFrameChild());
	t->mutable_time()->set_time(microTime);
	t->mutable_transform()->mutable_translation()->set_x(
			transform.getTranslation().x());
	t->mutable_transform()->mutable_translation()->set_y(
			transform.getTranslation().y());
	t->mutable_transform()->mutable_translation()->set_z(
			transform.getTranslation().z());
	t->mutable_transform()->mutable_rotation()->set_qw(
			transform.getRotationQuat().w());
	t->mutable_transform()->mutable_rotation()->set_qx(
			transform.getRotationQuat().x());
	t->mutable_transform()->mutable_rotation()->set_qy(
			transform.getRotationQuat().y());
	t->mutable_transform()->mutable_rotation()->set_qz(
			transform.getRotationQuat().z());
}
void TransformCommRsb::convertPbToTransform(
		const boost::shared_ptr<FrameTransform> &t, Transform& transform) {

	const boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
	const boost::posix_time::ptime time = epoch
			+ boost::posix_time::microseconds(t->time().time());

	Eigen::Vector3d p(t->transform().translation().x(),
			t->transform().translation().y(), t->transform().translation().z());
	Eigen::Quaterniond r(t->transform().rotation().qw(),
			t->transform().rotation().qx(), t->transform().rotation().qy(),
			t->transform().rotation().qz());
	Eigen::Affine3d a = Eigen::Affine3d().fromPositionOrientationScale(p, r,
			Eigen::Vector3d::Ones());

	transform.setFrameParent(t->frame_parent());
	transform.setFrameChild(t->frame_child());
	transform.setTime(time);
	transform.setTransform(a);

}
}  // namespace rct
