/*
 * TransformerRos.h
 *
 *  Created on: Mar 1, 2015
 *      Author: leon ziegler
 */

#include "TransformListenerRos.h"

using namespace rct;

TransformListenerRos::TransformListenerRos(TransformCallbackRos& cb) :
		clientCallback(cb) {
	ros::SubscribeOptions ops;
	ops.template initByFullCallbackType<const ros::MessageEvent<tf2_msgs::TFMessage const>&>("/tf",
			1000, boost::bind(&TransformListenerRos::subscriptionCallback, this, _1));
	subscriberTf = node.subscribe(ops);

	ros::SubscribeOptions ops_static;
	ops_static.template initByFullCallbackType<const ros::MessageEvent<tf2_msgs::TFMessage const>&>(
			"/tf_static", 1000,
			boost::bind(&TransformListenerRos::staticSubscriptionCallback, this, _1));
	subscriberTfStatic = node.subscribe(ops_static);
}

TransformListenerRos::~TransformListenerRos() {
}

void TransformListenerRos::subscriptionCallback(
		const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt) {
	subscriptionCallbackImpl(msg_evt, false);
}
void TransformListenerRos::staticSubscriptionCallback(
		const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt) {
	subscriptionCallbackImpl(msg_evt, true);
}

void TransformListenerRos::subscriptionCallbackImpl(
		const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt, bool is_static) {

	const tf2_msgs::TFMessage& msg_in = *(msg_evt.getConstMessage());
	std::string authority = msg_evt.getPublisherName(); // lookup the authority
	for (unsigned int i = 0; i < msg_in.transforms.size(); i++) {
		clientCallback(msg_in.transforms[i], authority, is_static);
	}
}
