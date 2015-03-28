/*
 * TransformCommRsb.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: leon
 */

#include "TransformCommRsb.h"
#include "TransformConverter.h"
#include <rsb/converter/Repository.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/Event.h>
#include <rsb/MetaData.h>
#include <rsc/runtime/TypeStringTools.h>
#include <log4cxx/log4cxx.h>

using namespace std;
using namespace rsb;
using namespace boost;

namespace rct {

rsc::logging::LoggerPtr TransformCommRsb::logger = rsc::logging::Logger::getLogger("rct.rsb.TransformComRsb");
string TransformCommRsb::defaultScopeSync = "/rct/sync";
string TransformCommRsb::defaultScopeTransforms = "/rct/transform";
string TransformCommRsb::defaultScopeSufficStatic = "/static";
string TransformCommRsb::defaultScopeSuffixDynamic = "/dynamic";
string TransformCommRsb::defaultUserKeyAuthority = "authority";

TransformCommRsb::TransformCommRsb(const string &authority) :
		authority(authority),
		scopeSync(defaultScopeSync),
		scopeTransforms(defaultScopeTransforms),
		scopeSuffixStatic(defaultScopeSufficStatic),
		scopeSuffixDynamic(defaultScopeSuffixDynamic),
		userKeyAuthority(defaultUserKeyAuthority) {
}

TransformCommRsb::TransformCommRsb(const string &authority, const TransformListener::Ptr& l) :
		authority(authority),
		scopeSync(defaultScopeSync),
		scopeTransforms(defaultScopeTransforms),
		scopeSuffixStatic(defaultScopeSufficStatic),
		scopeSuffixDynamic(defaultScopeSuffixDynamic),
		userKeyAuthority(defaultUserKeyAuthority) {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(const string &authority, const vector<TransformListener::Ptr>& l) :
		authority(authority),
		scopeSync(defaultScopeSync),
		scopeTransforms(defaultScopeTransforms),
		scopeSuffixStatic(defaultScopeSufficStatic),
		scopeSuffixDynamic(defaultScopeSuffixDynamic),
		userKeyAuthority(defaultUserKeyAuthority) {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(const string &authority, const TransformListener::Ptr& l,
		std::string scopeSync, std::string scopeTransforms, std::string scopeSuffixStatic,
		std::string scopeSuffixDynamic, std::string userKeyAuthority) :
		authority(authority),
		scopeSync(scopeSync),
		scopeTransforms(scopeTransforms),
		scopeSuffixStatic(scopeSuffixStatic),
		scopeSuffixDynamic(scopeSuffixDynamic),
		userKeyAuthority(userKeyAuthority) {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(const string &authority, const vector<TransformListener::Ptr>& l,
		std::string scopeSync, std::string scopeTransforms, std::string scopeSuffixStatic,
		std::string scopeSuffixDynamic, std::string userKeyAuthority) :
		authority(authority),
		scopeSync(scopeSync),
		scopeTransforms(scopeTransforms),
		scopeSuffixStatic(scopeSuffixStatic),
		scopeSuffixDynamic(scopeSuffixDynamic),
		userKeyAuthority(userKeyAuthority) {

	addTransformListener(l);
}

TransformCommRsb::~TransformCommRsb() {
}

void TransformCommRsb::init(const TransformerConfig &conf) {

	RSCDEBUG(logger, "init()");
	try {
		TransformConverter::Ptr converter0(new TransformConverter());
		converter::converterRepository<string>()->registerConverter(converter0);
	} catch (std::invalid_argument &e) {
		RSCTRACE(logger, "Converter already present");
	}

	Factory &factory = rsb::getFactory();

	rsbListenerTransform = factory.createListener(scopeTransforms);
	rsbListenerSync = factory.createListener(scopeSync);
	rsbInformerTransform = factory.createInformer<Transform>(scopeTransforms);
	rsbInformerSync = factory.createInformer<void>(scopeSync);

	EventFunction f0(bind(&TransformCommRsb::transformCallback, this, _1));
	transformHandler = HandlerPtr(new EventFunctionHandler(f0));
	rsbListenerTransform->addHandler(transformHandler);

	EventFunction f1(bind(&TransformCommRsb::triggerCallback, this, _1));
	syncHandler = HandlerPtr(new EventFunctionHandler(f1));
	rsbListenerSync->addHandler(syncHandler);

	requestSync();

}
void TransformCommRsb::shutdown() {
	listeners.clear();
	rsbListenerTransform->removeHandler(transformHandler);
	rsbListenerTransform->removeHandler(syncHandler);
}
void TransformCommRsb::requestSync() {

	if (!rsbInformerSync) {
		throw std::runtime_error("communicator was not initialized!");
	}

	RSCDEBUG(logger,
			"Sending sync request trigger from id " << rsbInformerSync->getId().getIdAsString());

	// trigger other instances to send transforms
	rsbInformerSync->publish(shared_ptr<void>());
}

bool TransformCommRsb::sendTransform(const Transform& transform, TransformType type) {
	if (!rsbInformerTransform) {
		throw std::runtime_error("communicator was not initialized!");
	}

	const string cacheKey = transform.getFrameParent() + transform.getFrameChild();

	RSCTRACE(logger, "sendTransform() ");

	MetaData meta;
	if (transform.getAuthority() == "") {
		meta.setUserInfo(userKeyAuthority, authority);
	} else {
		meta.setUserInfo(userKeyAuthority, transform.getAuthority());
	}

	boost::mutex::scoped_lock(mutex);
	RSCTRACE(logger,
			"Publishing transform from " << rsbInformerTransform->getId().getIdAsString());
	EventPtr event(rsbInformerTransform->createEvent());
	event->setData(make_shared<Transform>(transform));
	event->setMetaData(meta);

	if (type == STATIC) {
		sendCacheStatic[cacheKey] = make_pair(transform, meta);
		event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixStatic)));
	} else if (type == DYNAMIC) {
		sendCacheDynamic[cacheKey] = make_pair(transform, meta);
		event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixDynamic)));
	} else {
		RSCERROR(logger, "Cannot send transform. Reason: Unknown TransformType: " << type);
		return false;
	}
	RSCTRACE(logger, "sending " << event->getScope() << " " << transform);
	rsbInformerTransform->publish(event);
	RSCTRACE(logger, "sendTransform() done");
	return true;
}

bool TransformCommRsb::sendTransform(const std::vector<Transform>& transforms, TransformType type) {
	std::vector<Transform>::const_iterator it;
	for (it = transforms.begin(); it != transforms.end(); ++it) {
		sendTransform(*it, type);
	}
	return true;
}

void TransformCommRsb::publishCache() {
	RSCTRACE(logger, "Publishing cache from " << rsbInformerTransform->getId().getIdAsString());

	map<string, std::pair<Transform, MetaData> >::iterator it;
	for (it = sendCacheDynamic.begin(); it != sendCacheDynamic.end(); it++) {
		EventPtr event(rsbInformerTransform->createEvent());
		event->setData(make_shared<Transform>(it->second.first));
		event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixDynamic)));
		event->setMetaData(it->second.second);
		rsbInformerTransform->publish(event);
	}
	for (it = sendCacheStatic.begin(); it != sendCacheStatic.end(); it++) {
		EventPtr event(rsbInformerTransform->createEvent());
		event->setData(make_shared<Transform>(it->second.first));
		event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixStatic)));
		event->setMetaData(it->second.second);
		rsbInformerTransform->publish(event);
	}
}

void TransformCommRsb::addTransformListener(const TransformListener::Ptr& l) {
	boost::mutex::scoped_lock(mutex);
	listeners.push_back(l);
}

void TransformCommRsb::addTransformListener(const vector<TransformListener::Ptr>& l) {
	boost::mutex::scoped_lock(mutex);
	listeners.insert(listeners.end(), l.begin(), l.end());
}

void TransformCommRsb::removeTransformListener(const TransformListener::Ptr& l) {
	boost::mutex::scoped_lock(mutex);
	vector<TransformListener::Ptr>::iterator it = find(listeners.begin(), listeners.end(), l);
	if (it != listeners.end()) {
		listeners.erase(it);
	}
}

void TransformCommRsb::transformCallback(EventPtr event) {
	if (event->getMetaData().getSenderId() == rsbInformerTransform->getId()) {
		RSCTRACE(logger,
				"Received transform from myself. Ignore. (id " << event->getMetaData().getSenderId().getIdAsString() << ")");
		return;
	}

	boost::shared_ptr<Transform> t = boost::static_pointer_cast<Transform>(event->getData());
	string authority = event->getMetaData().getUserInfo(userKeyAuthority);

	Scope staticScope = rsbInformerTransform->getScope()->concat(Scope(scopeSuffixStatic));
	bool isStatic = (event->getScope() == staticScope);

	t->setAuthority(authority);
	RSCDEBUG(logger, "Received transform from " << authority);
	RSCTRACE(logger, "Received transform: " << *t);

	boost::mutex::scoped_lock(mutex);
	vector<TransformListener::Ptr>::iterator it0;
	for (it0 = listeners.begin(); it0 != listeners.end(); ++it0) {
		TransformListener::Ptr l = *it0;
		l->newTransformAvailable(*t, isStatic);
	}
}

void TransformCommRsb::triggerCallback(EventPtr e) {

	if (e->getMetaData().getSenderId() == rsbInformerSync->getId()) {
		RSCTRACE(logger,
				"Got sync request from myself. Ignore. (id " << e->getMetaData().getSenderId().getIdAsString() << ")");
		return;
	}

	// publish send cache as thread
	boost::thread workerThread(&TransformCommRsb::publishCache, this);
}

void TransformCommRsb::printContents(std::ostream& stream) const {
	stream << "authority = " << authority;
	stream << ", communication = rsb";
	stream << ", #listeners = " << listeners.size();
	stream << ", #cache = " << sendCacheDynamic.size();
}

string TransformCommRsb::getAuthorityName() const {
	return authority;
}
}  // namespace rct
