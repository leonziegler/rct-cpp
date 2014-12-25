/*
 * RctEcho.cpp
 *
 *  Created on: Dec 23, 2014
 *      Author: leon
 */
#include <rct/rctConfig.h>
#include <rct/TransformerFactory.h>
#include <rct/Transformer.h>
#include <boost/program_options.hpp>
#include <log4cxx/log4cxx.h>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>

using namespace boost::program_options;
using namespace std;
using namespace log4cxx;

void printHelp(int argc, char **argv, options_description desc) {
	cout << "Usage:\n  " << argv[0] << " [options] source_frame target_frame\n"
			<< endl;
	cout << desc << endl;
	cout << "This will echo the transform from the coordinate" << endl;
	cout << "frame of the source_frame to the coordinate frame" << endl;
	cout << "of the target_frame." << endl;
	cout << "Note: This is the transform to get data from" << endl;
	cout << "target_frame into the source_frame." << endl;
}

int handleArgs(int argc, char **argv, string &frame_target,
		string &frame_source, bool &matrix, bool &quaternion) {
	boost::program_options::positional_options_description p0;
	p0.add("frames", -1);

	options_description desc("Allowed options");
	variables_map vm;

	desc.add_options()("help,h", "produce help message") // help
	("debug", "debug mode") //debug
	("trace", "trace mode") //trace
	("matrix", "print transformation matrix") //matrix
	("quaternion", "print rotation as quaternion") //quaternion
	("info", "info mode");

	options_description hidden("Hidden options");
	hidden.add_options()("frames", value<vector<string> >(), "source frame");

	options_description all;
	all.add(desc).add(hidden);
	store(command_line_parser(argc, argv).options(all).positional(p0).run(),
			vm);
	notify(vm);

	if (vm.count("help")) {
		cout << 0 << endl;
		printHelp(argc, argv, desc);
		return 0;
	}

	LayoutPtr pattern(new PatternLayout("%r [%t] %-5p %c - %m%n"));
	AppenderPtr appender(new ConsoleAppender(pattern));
	Logger::getRootLogger()->addAppender(appender);

	if (vm.count("debug")) {
		Logger::getRootLogger()->setLevel(Level::getDebug());
	} else if (vm.count("trace")) {
		Logger::getRootLogger()->setLevel(Level::getTrace());
	} else if (vm.count("info")) {
		Logger::getRootLogger()->setLevel(Level::getInfo());
	} else {
		Logger::getRootLogger()->setLevel(Level::getWarn());
	}

	if (!vm.count("frames")) {
		cout << 1 << endl;
		printHelp(argc, argv, desc);
		return -1;
	}

	vector<string> frames = vm["frames"].as<vector<string> >();
	if (frames.size() != 2) {
		cout << 2 << endl;
		printHelp(argc, argv, desc);
		return -1;
	}

	frame_source = frames[0];
	frame_target = frames[1];
	matrix = vm.count("matrix");
	quaternion = vm.count("quaternion");
	return 0;
}

int main(int argc, char **argv) {

	string frame_target, frame_source;
	bool matrix, quaternion;

	int ret = handleArgs(argc, argv, frame_target, frame_source, matrix,
			quaternion);
	if (ret != 0) {
		return ret;
	}

	boost::posix_time::ptime now(
			boost::posix_time::microsec_clock::universal_time());

	rct::Transformer::Ptr transformerRsb =
			rct::getTransformerFactory().createTransformer();
	rct::Transformer::FuturePtr future;
	future = transformerRsb->requestTransform(frame_target, frame_source, now);

	try {
		rct::Transform t = future->get(2.0);

		if (matrix) {
			cout << "Transformation:\n" << t.getTransform().matrix() << endl;
		} else if (quaternion) {
			cout << "Translation (x,y,z):\n" << t.getTranslation() << endl;
			cout << "Rotation (Quat w,x,y,z):\n" << t.getRotationQuat().w() << "\n"
					<< t.getRotationQuat().x() << "\n" << t.getRotationQuat().y()
					<< "\n" << t.getRotationQuat().z() << endl;
		} else {
			Eigen::Vector3d ypr = t.getRotationYPR();
			cout << "Translation (x,y,z):\n" << t.getTranslation() << endl;
			cout << "Rotation (yaw,pitch,roll):\n" << ypr.x() << "\n" << ypr.y()
					<< "\n" << ypr.z() << endl;
		}
		return 0;
	} catch (std::exception &e) {
		cerr << "ERROR: " << e.what() << endl;
		return 1;
	}
}
