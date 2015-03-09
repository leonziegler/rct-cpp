/*
 * publisher.cpp
 *
 *  Created on: Dec 21, 2014
 *      Author: leon
 */

#include <csignal>
#include <rct/TransformerFactory.h>

bool enabled = true;

void int_handler(int x) {
	enabled = false;
}

int main(int argc, char **argv) {

	signal(SIGINT, int_handler);

	// create the transformer object
	rct::Transformer::Ptr transformer = rct::getTransformerFactory().createTransformer("ExamplePublisher");

	// create a static transform
	Eigen::Vector3d position(0.0, 1.0, 2.0);
	Eigen::AngleAxisd orientation(M_PI, Eigen::Vector3d::UnitX());
	Eigen::Affine3d affine = Eigen::Affine3d().fromPositionOrientationScale(position, orientation,
						Eigen::Vector3d::Ones());
	rct::Transform staticTransform(affine, "A", "B", boost::posix_time::microsec_clock::universal_time());

	// publish the static transform
	transformer->sendTransform(staticTransform, rct::STATIC);

	double angle = 0;
	while(enabled) {

		// alter angle
		angle += 0.01;
		if (angle > 2 * M_PI) angle = 0;

		// create a dynamic transform
		Eigen::Vector3d position(0.0, 1.0, 2.0);
		Eigen::AngleAxisd orientation(angle, Eigen::Vector3d::UnitX());
		Eigen::Affine3d affine = Eigen::Affine3d().fromPositionOrientationScale(position, orientation,
							Eigen::Vector3d::Ones());
		rct::Transform dynamicTransform(affine, "B", "C", boost::posix_time::microsec_clock::universal_time());

		// publish the dynamic transform
		transformer->sendTransform(dynamicTransform, rct::DYNAMIC);

		usleep(20 * 1000); // 50hz
	}
}


