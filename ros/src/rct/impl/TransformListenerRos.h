/*
 * TransformerRos.h
 *
 *  Created on: Mar 1, 2015
 *      Author: leon ziegler
 */

#pragma once

#include <std_msgs/Empty.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/ros.h>

#include <tf2/buffer_core.h>

#include <boost/thread.hpp>

namespace rct {

typedef boost::function<void(const geometry_msgs::TransformStamped transform, const std::string & authority, bool is_static)> TransformCallbackRos;

class TransformListenerRos {
public:
	TransformListenerRos(TransformCallbackRos& cb);
	~TransformListenerRos();

private:

  void subscriptionCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt);
  void staticSubscriptionCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt);
  void subscriptionCallbackImpl(const ros::MessageEvent<tf2_msgs::TFMessage const>& msg_evt, bool is_static);

  TransformCallbackRos clientCallback;
  ros::NodeHandle node;
  ros::Subscriber subscriberTf;
  ros::Subscriber subscriberTfStatic;
};
}
