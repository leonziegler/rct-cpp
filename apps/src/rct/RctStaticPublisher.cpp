/*
 * RctStaticPublisher.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author: leon
 */

#include "RctStaticPublisher.h"
#include <rct/rctConfig.h>

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <log4cxx/log4cxx.h>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>
#include <iostream>
#include <csignal>

using namespace boost::program_options;
using namespace boost::property_tree;
using namespace boost::filesystem;
using namespace std;
using namespace log4cxx;

rct::RctStaticPublisher *publisher;

void signalHandler(int signum) {
	cout << "Interrupt signal (" << signum << ") received." << endl;
	publisher->interrupt();
}

int main(int argc, char **argv) {
	options_description desc("Allowed options");
	variables_map vm;

	desc.add_options()("help,h", "produce help message") // help
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

	try {

		publisher = new rct::RctStaticPublisher(vm["config"].as<string>(), vm.count("bridge"));

		// register signal SIGINT and signal handler
		signal(SIGINT, signalHandler);

		// block
		publisher->run();

	} catch (std::exception &e) {
		cerr << "Error:\n  " << e.what() << "\n" << endl;
		return 1;
	}

	return 0;
}

namespace rct {

log4cxx::LoggerPtr RctStaticPublisher::logger = log4cxx::Logger::getLogger("rct.RctStaticPublisher");

RctStaticPublisher::RctStaticPublisher(const string &configFile, bool bridge) :
		configFile(configFile), bridge(bridge), interrupted(false) {

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

	LOG4CXX_DEBUG(logger, "reading config file: " << configFile)
	vector<Transform> transforms = parseTransforms(configFile);
	transformerRsb->sendTransform(transforms, true);

	// run until interrupted
	while (!interrupted) {
		boost::mutex::scoped_lock lock(mutex);
		// wait for notification
		cond.wait(lock);
		LOG4CXX_DEBUG(logger, "notified");
		if (bridge) {
			while(rsbHandler->hasTransforms()) {
				TransformWrapper t = rsbHandler->nextTransform();
				if (t.getAuthority() != transformerRsb->getAuthorityName()) {
					commRos->sendTransform(t, t.isStatic);
				} else {
					LOG4CXX_TRACE(logger, "skip bridging of transform from rsb to ros because own authority: " << t.getAuthority());
				}
			}
			while(rosHandler->hasTransforms()) {
				TransformWrapper t = rosHandler->nextTransform();
				if (t.getAuthority() != commRos->getAuthorityName()) {
					transformerRsb->sendTransform(t, t.isStatic);
				} else {
					LOG4CXX_TRACE(logger, "skip bridging of transform from ros to rsb because own authority: " << t.getAuthority());
				}
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

void Handler::newTransformAvailable(const Transform& transform, bool isStatic) {
	boost::mutex::scoped_lock lock(mutex);
	TransformWrapper w(transform, isStatic);
	transforms.push_back(w);
	parent->notify();
}
bool Handler::hasTransforms() {
	boost::mutex::scoped_lock lock(mutex);
	return !transforms.empty();
}

TransformWrapper Handler::nextTransform() {
	if (!hasTransforms()) {
		throw std::range_error("no transforms available");
	}
	boost::mutex::scoped_lock lock(mutex);
	TransformWrapper ret = *transforms.begin();
	transforms.erase(transforms.begin());
	return ret;
}

vector<Transform> RctStaticPublisher::parseTransforms(const string& file) const {
	ptree pt;
	ini_parser::read_ini(file, pt);

	vector<Transform> transforms;
	ptree::const_iterator itTrans;
	for (itTrans = pt.begin(); itTrans != pt.end(); ++itTrans) {
		if (!boost::algorithm::starts_with(itTrans->first, "transform")) {
			continue;
		}
		string section = itTrans->first;
		ptree ptTransform = itTrans->second;

		string parent = ptTransform.get<string>("parent");
		string child = ptTransform.get<string>("child");
		boost::optional<double> tx = ptTransform.get_optional<double>(boost::property_tree::path("translation.x", '/'));
		boost::optional<double> ty = ptTransform.get_optional<double>(boost::property_tree::path("translation.y", '/'));
		boost::optional<double> tz = ptTransform.get_optional<double>(boost::property_tree::path("translation.z", '/'));
		boost::optional<double> yaw =   ptTransform.get_optional<double>(boost::property_tree::path("rotation.yaw", '/'));
		boost::optional<double> pitch = ptTransform.get_optional<double>(boost::property_tree::path("rotation.pitch", '/'));
		boost::optional<double> roll =  ptTransform.get_optional<double>(boost::property_tree::path("rotation.roll", '/'));
		boost::optional<double> qw = ptTransform.get_optional<double>(boost::property_tree::path("rotation.qw", '/'));
		boost::optional<double> qx = ptTransform.get_optional<double>(boost::property_tree::path("rotation.qx", '/'));
		boost::optional<double> qy = ptTransform.get_optional<double>(boost::property_tree::path("rotation.qy", '/'));
		boost::optional<double> qz = ptTransform.get_optional<double>(boost::property_tree::path("rotation.qz", '/'));

		if (!tx || !ty || !tz) {
			stringstream ss;
			ss << "Error parsing transforms. ";
			ss << "Section \"" << section << "\" has incomplete translation. ";
			ss << "Required: (translation.x, translation.y, translation.z)";
			throw ptree_error(ss.str());
		}

		Eigen::Vector3d translation(tx.get(), ty.get(), tz.get());

		if (yaw && pitch && roll) {
			if (qw || qx || qy || qz) {
				stringstream ss;
				ss << "Error parsing transforms. ";
				ss << "Section \"" << section << "\" has arbitrary rotation declarations. ";
				ss << "Use either yaw/pitch/roll or quaternion. ";
				throw ptree_error(ss.str());
			}
			Eigen::AngleAxisd rollAngle(roll.get(), Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd yawAngle(yaw.get(), Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd pitchAngle(pitch.get(), Eigen::Vector3d::UnitX());
			Eigen::Quaterniond r = rollAngle * yawAngle * pitchAngle;
			Eigen::Affine3d a = Eigen::Affine3d().fromPositionOrientationScale(translation, r,
								Eigen::Vector3d::Ones());
			Transform t(a, parent, child, boost::posix_time::microsec_clock::universal_time());
			LOG4CXX_DEBUG(logger, "parsed transform: " << t);
			transforms.push_back(t);
			continue;

		} else if (qw && qx && qy && qz) {
			if (yaw || pitch || roll) {
				stringstream ss;
				ss << "Error parsing transforms. ";
				ss << "Section \"" << section << "\" has arbitrary rotation declarations. ";
				ss << "Use either yaw/pitch/roll or quaternion. ";
				throw ptree_error(ss.str());
			}
			Eigen::Quaterniond r(qw.get(), qx.get(), qy.get(), qz.get());
			Eigen::Affine3d a = Eigen::Affine3d().fromPositionOrientationScale(translation, r,
								Eigen::Vector3d::Ones());
			Transform t(a, parent, child, boost::posix_time::microsec_clock::universal_time());
			LOG4CXX_DEBUG(logger, "parsed transform: " << t);
			transforms.push_back(t);
			continue;
		} else {
			stringstream ss;
			ss << "Error parsing transforms. ";
			ss << "Section \"" << itTrans->first << "\" has incomplete rotation. ";
			ss << "Required: (rotation.yaw, rotation.pitch, rotation.roll)";
			ss << " OR (rotation.qw, rotation.qx, rotation.qy, rotation.qz)";
			throw ptree_error(ss.str());
		}
	}
	return transforms;
}

} /* namespace rct */
