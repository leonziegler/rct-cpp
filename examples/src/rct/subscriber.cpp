/*
 * subscriber.cpp
 *
 *  Created on: Dec 21, 2014
 *      Author: leon
 */

#include <rct/TransformerFactory.h>
#include <iostream>

int main(int argc, char **argv) {

	rct::TransformReceiver::Ptr receiver = rct::getTransformerFactory().createTransformReceiver();

	std::cout << "###\n### first lookup (relevant information not available yet)\n###" << std::endl;

	try {
	    // lookup a transform which describes the status for the target system at the
	    // current point in time ("right now"). This will fail because the required information is
	    // not available yet.
		rct::Transform t = receiver->lookupTransform("A", "C",
				boost::posix_time::microsec_clock::universal_time());

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "\n###\n### first request (wait for transform using future object)\n###" << std::endl;

	try {
	    // request transform using a future object which will be filled as soon as the transform
	    // is available. This allows waiting for the result.
		rct::TransformReceiver::FuturePtr future = receiver->requestTransform("A", "C",
				boost::posix_time::microsec_clock::universal_time());

		rct::Transform t = future->get(2.0);

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "\n###\n### second lookup (lookup most recent transform)\n###" << std::endl;

	try {
	    // lookup the most recent available transform for the requested frames.
		rct::Transform t = receiver->lookupTransform("A", "C", boost::posix_time::from_time_t(0));

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "\n###\n### third lookup (lookup in the past)\n###" << std::endl;

	try {
		// wait 20 milliseconds
		usleep(20*1000);

		// lookup at now minus 20 milliseconds
		rct::Transform t = receiver->lookupTransform("A", "C",
				boost::posix_time::microsec_clock::universal_time() - boost::posix_time::time_duration(0, 0, 0, 20*1000));

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	return 0;
}

