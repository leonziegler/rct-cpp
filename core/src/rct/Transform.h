/*
 * Transform.h
 *
 *  Created on: Dec 7, 2014
 *      Author: leon
 */

#pragma once

#include <Eigen/Geometry>
#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
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

	const Eigen::Vector3d getRotationYPR() const {

		Eigen::Matrix3d mat = transform.rotation().matrix();

    	// this code is taken from buttel btMatrix3x3 getEulerYPR().
    	// http://bulletphysics.org/Bullet/BulletFull/btMatrix3x3_8h_source.html
		// first use the normal calculus
		double yawOut = atan2(mat(1,0), mat(0,0));
		double pitchOut = asin(-mat(2,0));
		double rollOut = atan2(mat(2,1), mat(2,2));

		// on pitch = +/-HalfPI
		if (abs(pitchOut) == M_PI / 2.0) {
			if (yawOut > 0)
				yawOut -= M_PI;
			else
				yawOut += M_PI;
			if (pitchOut > 0)
				pitchOut -= M_PI;
			else
				pitchOut += M_PI;
		}

		return Eigen::Vector3d(yawOut, pitchOut, rollOut);
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

		Eigen::IOFormat commaFmt(2, Eigen::DontAlignCols, ",", ";", "", "", "[", "]");

		stream << "authority = " << authority;
		stream << ", frameParent = " << frameParent;
		stream << ", frameChild = " << frameChild;
		stream << ", time = " << time;
		stream << ", transform = " << transform.matrix().format(commaFmt);
	}

private:
	Eigen::Affine3d transform;
	std::string frameParent;
	std::string frameChild;
	boost::posix_time::ptime time;
	std::string authority;
};
}
