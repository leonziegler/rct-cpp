/*
 * RctStaticPublisher.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author: leon
 */

#include "RctStaticPublisher.h"
#include <rct/rctConfig.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <log4cxx/log4cxx.h>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>
#include <iostream>
#include <csignal>

using namespace boost::program_options;
using namespace boost::filesystem;
using namespace std;
using namespace log4cxx;

rct::RctStaticPublisher *publisher;

vector<string> readDir(const string &dir) {
	vector<string> vec;
	path someDir(dir);
	directory_iterator end_iter;

	if (exists(someDir) && is_directory(someDir)) {
		for (directory_iterator dir_iter(someDir); dir_iter != end_iter;
				++dir_iter) {
			if (is_regular_file(dir_iter->status())) {
				path f(*dir_iter);
				stringstream ss;
				ss << dir << "/" << f.filename();
				vec.push_back(ss.str());
			}
		}
	}
	return vec;
}

void signalHandler(int signum) {
	cout << "Interrupt signal (" << signum << ") received." << endl;
	publisher->interrupt();
}

int main(int argc, char **argv) {
	options_description desc("Allowed options");
	variables_map vm;

	desc.add_options()("help,h", "produce help message") // help
	("dir,d", value<string>(), "directory containing config files") // config dir
	("config,c", value<string>(), "a single config file") // config file
	("bridge", "rsb/ros bride mode") //bridge
	("debug", "debug mode") //debug
	("trace", "trace mode") //trace
	("info", "info mode");

	store(parse_command_line(argc, argv, desc), vm);
	notify(vm);

	if (vm.count("help")) {
		cout << "Usage:\n  " << argv[0] << " [options]\n" << endl;
		cout << desc << endl;
		return 0;
	}

	if (!vm.count("dir") && !vm.count("config")) {
		cerr << "\nERROR: either --dir or --config must be set!" << endl
				<< endl;
		cout << desc << endl;
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

	vector<string> files;
	if (vm.count("dir")) {
		files = readDir(vm["dir"].as<string>());
	}
	if (vm.count("config")) {
		files.push_back(vm["config"].as<string>());
	}

	publisher = new rct::RctStaticPublisher(files, vm.count("bridge"));

	// register signal SIGINT and signal handler
	signal(SIGINT, signalHandler);

	// block
	publisher->run();

	return 0;
}

namespace rct {

log4cxx::LoggerPtr RctStaticPublisher::logger = log4cxx::Logger::getLogger("RctStaticPublisher");

RctStaticPublisher::RctStaticPublisher(const vector<string> &configFiles,
		bool bridge) :
		bridge(bridge), interrupted(false) {

	TransformerConfig configRsb;
	configRsb.setCommType(TransformerConfig::RSB);

	if (bridge) {
#ifdef RCT_HAVE_ROS
		rosHandler = Handler::Ptr(new Handler(this));
		rsbHandler = Handler::Ptr(new Handler(this));

		transformerRsb = getTransformerFactory().createTransformer(rsbHandler, configRsb);

		TransformerConfig configRos;
		configRos.setCommType(TransformerConfig::ROS);
		commRos = TransformCommRos::Ptr(new TransformCommRos(configRos.getCacheTime(), rosHandler));
#else
		throw TransformerFactoryException("Can not activate bridge mode, because ROS implementation is not present!");
#endif
	} else {
		transformerRsb = getTransformerFactory().createTransformer(configRsb);
	}
}

void RctStaticPublisher::notify() {
	boost::mutex::scoped_lock lock(mutex);
	cond.notify_all();
}

void RctStaticPublisher::run() {

	// run until interrupted
	while (!interrupted) {
		boost::mutex::scoped_lock lock(mutex);
		// wait for notification
		cond.wait(lock);
		LOG4CXX_DEBUG(logger, "notified");
		if (bridge) {
			while(rsbHandler->hasTransforms()) {
				Transform t = rsbHandler->nextTransform();
				commRos->sendTransform(t);
				// TODO inf loop
			}
			while(rosHandler->hasTransforms()) {
				Transform t = rosHandler->nextTransform();
				transformerRsb->sendTransform(t);
				// TODO inf loop
			}
		}
	}
	LOG4CXX_DEBUG(logger, "interrupted");
}
void RctStaticPublisher::interrupt() {
	interrupted = true;
	notify();
}
RctStaticPublisher::~RctStaticPublisher() {
}

void Handler::newTransformAvailable(const Transform& transform) {
	boost::mutex::scoped_lock lock(mutex);
	transforms.push_back(transform);
	parent->notify();
}
bool Handler::hasTransforms() {
	boost::mutex::scoped_lock lock(mutex);
	return !transforms.empty();
}
Transform Handler::nextTransform() {
	if (!hasTransforms()) {
		throw std::range_error("no transforms available");
	}
	boost::mutex::scoped_lock lock(mutex);
	Transform ret = *transforms.begin();
	transforms.erase(transforms.begin());
	return ret;
}

} /* namespace rct */
