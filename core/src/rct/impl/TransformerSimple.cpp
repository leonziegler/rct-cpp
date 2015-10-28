/*
 * TransformerSimple.cpp
 *
 *  Created on: 19.10.2015
 *      Author: leon
 */

#include "../impl/TransformerSimple.h"
#include "../Exceptions.h"

#include <boost/algorithm/string.hpp>

using namespace boost;
using namespace std;
using namespace Eigen;

namespace rct {

rsc::logging::LoggerPtr TransformerSimple::logger = rsc::logging::Logger::getLogger("rct.core.TransformerSimple");

TransformerSimple::TransformerSimple(const posix_time::time_duration& cacheTime) :
		tfBuffer(cacheTime) {

	tfBuffer.addTransformsChangedListener(bind(&TransformerSimple::treeChanged, this));
}

TransformerSimple::~TransformerSimple() {
}

void TransformerSimple::clear() {
	tfBuffer.clear();
}

bool TransformerSimple::setTransform(const Transform& transform_in, bool is_static) {

	return tfBuffer.setTransform(transform_in, is_static);
}

Transform TransformerSimple::lookupTransform(const std::string& target_frame,
		const std::string& source_frame, const posix_time::ptime& time) const {

	return tfBuffer.lookupTransform(target_frame, source_frame, time);
}

Transform TransformerSimple::lookupTransform(const std::string& target_frame,
		const posix_time::ptime& target_time, const std::string& source_frame,
		const posix_time::ptime& source_time,
		const std::string& fixed_frame) const {
	return tfBuffer.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

TransformerSimple::FuturePtr TransformerSimple::requestTransform(
		const std::string& target_frame, const std::string& source_frame,
		const boost::posix_time::ptime& time) {

	FuturePtr result(new FutureType());

	boost::mutex::scoped_lock lock(inprogressMutex);
	try {
		Transform t = lookupTransform(target_frame, source_frame, time);
		result->set(t);
		RSCTRACE(this->logger, "Lookup possible before request applies. Take shortcut.");

	} catch (LookupException &e) {

		RSCTRACE(this->logger, "Lookup NOT possible before request applies. Register request.");

		this->requestsInProgress.insert(
				std::make_pair(Request(target_frame, source_frame, time),
						result));
	} catch (ExtrapolationException &e) {

		RSCTRACE(this->logger, "Lookup NOT possible before request applies. Register request.");

		this->requestsInProgress.insert(
				std::make_pair(Request(target_frame, source_frame, time), result));
	}

	return result;
}

void TransformerSimple::treeChanged() {
	boost::mutex::scoped_lock lock(inprogressMutex);

	if (firstChangeTime.is_not_a_date_time()) {
		boost::posix_time::ptime now(boost::posix_time::microsec_clock::universal_time());
		firstChangeTime = boost::posix_time::ptime(boost::posix_time::microsec_clock::universal_time());
	}

	std::map<Request, FuturePtr>::iterator it;
	for (it = requestsInProgress.begin(); it != requestsInProgress.end(); ++it) {
		try {
		    boost::posix_time::ptime t = it->first.time;
			if (t < firstChangeTime) {
				t = firstChangeTime;
			}
			Transform t0 = tfBuffer.lookupTransform(it->first.target_frame,
					it->first.source_frame, t);
			it->second->set(t0);
			requestsInProgress.erase(it);
		} catch (LookupException &e) {
			RSCTRACE(this->logger, "Not yet transformable ");
		} catch (ExtrapolationException &e) {
			RSCTRACE(this->logger, "Not yet transformable ");
		}
	}
}

bool TransformerSimple::canTransform(const std::string& target_frame, const std::string& source_frame,
		const posix_time::ptime& time) const {
	return tfBuffer.canTransform(target_frame, source_frame, time);
}

bool TransformerSimple::canTransform(const std::string& target_frame,
		const posix_time::ptime& target_time, const std::string& source_frame,
		const posix_time::ptime& source_time, const std::string& fixed_frame) const {
	return tfBuffer.canTransform(target_frame, target_time, source_frame,
			source_time, fixed_frame);
}

void TransformerSimple::newTransformAvailable(const rct::Transform& t, bool isStatic) {
	setTransform(t, isStatic);
}

void TransformerSimple::printContents(std::ostream& stream) const {
	stream << "backend = FrameTreeSimple";
}

std::vector<std::string> TransformerSimple::getFrameStrings() const {
	return tfBuffer.getFrameStrings();
}

bool TransformerSimple::frameExists(const std::string& frame_id_str) const {
	return tfBuffer.frameExists(frame_id_str);
}

std::string TransformerSimple::getParent(const std::string& frame_id,
		const boost::posix_time::ptime &time) const {
	return tfBuffer.getParent(frame_id, time);
}

std::string TransformerSimple::allFramesAsDot() const {
	return tfBuffer.allFramesAsDot();
}

std::string TransformerSimple::allFramesAsYAML() const {
	return tfBuffer.allFramesAsYAML();
}

std::string TransformerSimple::allFramesAsString() const {
	return tfBuffer.allFramesAsString();
}

} /* namespace rct */

