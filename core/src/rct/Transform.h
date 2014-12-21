/*
 * Transform.h
 *
 *  Created on: Dec 7, 2014
 *      Author: leon
 */

#pragma once

#include <Eigen/Geometry>
#include <boost/date_time.hpp>
#include <rsc/runtime/Printable.h>

namespace rct {

class Transform: public rsc::runtime::Printable {
public:
	Transform() {
	}
	Transform(const Eigen::Affine3d &transform, const std::string &frameParent,
			const std::string &frameChild, const boost::posix_time::ptime &time) :
			transform(transform), frameParent(frameParent), frameChild(
					frameChild), time(time) {
	}
	virtual ~Transform() {
	}

	const std::string& getFrameChild() const {
		return frameChild;
	}

	void setFrameChild(const std::string& frameChild) {
		this->frameChild = frameChild;
	}

	const std::string& getFrameParent() const {
		return frameParent;
	}

	void setFrameParent(const std::string& frameParent) {
		this->frameParent = frameParent;
	}

	const boost::posix_time::ptime& getTime() const {
		return time;
	}

	void setTime(const boost::posix_time::ptime& time) {
		this->time = time;
	}

	void setAuthority(const std::string &authority) {
		this->authority = authority;
	}

	const Eigen::Affine3d& getTransform() const {
		return transform;
	}

	void setTransform(const Eigen::Affine3d& transform) {
		this->transform = transform;
	}

	const Eigen::Vector3d getTranslation() const {
		return transform.translation();
	}

	const Eigen::Quaterniond getRotationQuat() const {
		Eigen::Quaterniond quat(transform.rotation().matrix());
		return quat;
	}

	const Eigen::Matrix3d getRotationMatrix() const {
		return transform.rotation().matrix();
	}

	const std::string getAuthority() const {
		return authority;
	}

	virtual std::string getClassName() const {
		return "Transform";
	}
	virtual void printContents(std::ostream& stream) const {
		stream << "authority = " << authority;
		stream << ", frameParent = " << frameParent;
		stream << ", frameChild = " << frameChild;
		stream << ", time = " << time;
		stream << ", transform = " << transform.matrix();
	}

private:
	Eigen::Affine3d transform;
	std::string frameParent;
	std::string frameChild;
	boost::posix_time::ptime time;
	std::string authority;
};
}
