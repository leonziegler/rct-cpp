/*
 * subscriber.cpp
 *
 *  Created on: Dec 21, 2014
 *      Author: leon
 */

#include <rct/TransformerFactory.h>
#include <iostream>

int main(int argc, char **argv) {

	rct::Transformer::Ptr transformer = rct::getTransformerFactory().createTransformer(
			"SubscriberExample");

	std::cout << "###\n### first lookup\n###" << std::endl;

	try {
		rct::Transform t = transformer->lookupTransform("A", "C",
				boost::posix_time::microsec_clock::universal_time());

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "\n###\n### first request\n###" << std::endl;

	try {
		rct::Transformer::FuturePtr future = transformer->requestTransform("A", "C",
				boost::posix_time::microsec_clock::universal_time());

		rct::Transform t = future->get(2.0);

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "\n###\n### second lookup\n###" << std::endl;

	try {
		rct::Transform t = transformer->lookupTransform("A", "C",
				boost::posix_time::microsec_clock::universal_time());

		std::cout << "Translation (x,y,z):\n" << t.getTranslation() << std::endl;
		std::cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
				<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y() << "\n"
				<< t.getRotationQuat().z() << std::endl;
	} catch (std::exception &e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "\n###\n### third lookup\n###" << std::endl;

	try {
		// wait 20 milliseconds
		usleep(20*1000);

		// lookup at now minus 20 milliseconds
		rct::Transform t = transformer->lookupTransform("A", "C",
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

