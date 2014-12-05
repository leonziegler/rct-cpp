/*
 * Transformer.cpp
 *
 *  Created on: 05.12.2014
 *      Author: leon
 */

#include "Transformer.h"

using namespace boost;
using namespace std;

namespace rct {

Transformer::Transformer() {

}

Transformer::~Transformer() {
}

void Transformer::clear() {


}

bool Transformer::setTransform(
		const geometry_msgs::TransformStamped& transform_in,
		const std::string& authority, bool is_static) {

}

geometry_msgs::TransformStamped Transformer::lookupTransform(
		const std::string& target_frame, const std::string& source_frame,
		const ros::Time& time) const {

}

geometry_msgs::TransformStamped Transformer::lookupTransform(
		const std::string& target_frame, const ros::Time& target_time,
		const std::string& source_frame, const ros::Time& source_time,
		const std::string& fixed_frame) const {

}

bool Transformer::canTransform(const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		std::string* error_msg) const {

}

bool Transformer::canTransform(const std::string& target_frame,
		const ros::Time& target_time, const std::string& source_frame,
		const ros::Time& source_time, const std::string& fixed_frame,
		std::string* error_msg) const {

}

} /* namespace rct */
