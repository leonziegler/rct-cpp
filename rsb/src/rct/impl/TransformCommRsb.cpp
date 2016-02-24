/*
 * TransformCommRsb.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: leon
 */

#include "TransformCommRsb.h"
#include "TransformConverter.h"
#include "TransformCollectionConverter.h"
#include <rsb/converter/Repository.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/Event.h>
#include <rsb/MetaData.h>
#include <rsc/runtime/TypeStringTools.h>
#include <log4cxx/log4cxx.h>
#include <boost/make_shared.hpp>

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

TransformCommRsb::TransformCommRsb(const string &authority, bool legacyMode) :
		authority(authority),
		scopeSync(defaultScopeSync),
		scopeTransforms(defaultScopeTransforms),
		scopeSuffixStatic(defaultScopeSufficStatic),
		scopeSuffixDynamic(defaultScopeSuffixDynamic),
		userKeyAuthority(defaultUserKeyAuthority),
                legacyMode(legacyMode) {
}

TransformCommRsb::TransformCommRsb(const string &authority, const TransformListener::Ptr& l, bool legacyMode) :
		authority(authority),
		scopeSync(defaultScopeSync),
		scopeTransforms(defaultScopeTransforms),
		scopeSuffixStatic(defaultScopeSufficStatic),
		scopeSuffixDynamic(defaultScopeSuffixDynamic),
		userKeyAuthority(defaultUserKeyAuthority),
                legacyMode(legacyMode)  {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(const string &authority, const vector<TransformListener::Ptr>& l, bool legacyMode) :
		authority(authority),
		scopeSync(defaultScopeSync),
		scopeTransforms(defaultScopeTransforms),
		scopeSuffixStatic(defaultScopeSufficStatic),
		scopeSuffixDynamic(defaultScopeSuffixDynamic),
		userKeyAuthority(defaultUserKeyAuthority),
                legacyMode(legacyMode)  {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(const string &authority, const TransformListener::Ptr& l,
		std::string scopeSync, std::string scopeTransforms, std::string scopeSuffixStatic,
		std::string scopeSuffixDynamic, std::string userKeyAuthority, bool legacyMode) :
		authority(authority),
		scopeSync(scopeSync),
		scopeTransforms(scopeTransforms),
		scopeSuffixStatic(scopeSuffixStatic),
		scopeSuffixDynamic(scopeSuffixDynamic),
		userKeyAuthority(userKeyAuthority),
                legacyMode(legacyMode)  {

	addTransformListener(l);
}

TransformCommRsb::TransformCommRsb(const string &authority, const vector<TransformListener::Ptr>& l,
		std::string scopeSync, std::string scopeTransforms, std::string scopeSuffixStatic,
		std::string scopeSuffixDynamic, std::string userKeyAuthority, bool legacyMode) :
		authority(authority),
		scopeSync(scopeSync),
		scopeTransforms(scopeTransforms),
		scopeSuffixStatic(scopeSuffixStatic),
		scopeSuffixDynamic(scopeSuffixDynamic),
		userKeyAuthority(userKeyAuthority),
                legacyMode(legacyMode)  {

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
	try {
		TransformCollectionConverter::Ptr converter0(new TransformCollectionConverter());
		converter::converterRepository<string>()->registerConverter(converter0);
	} catch (std::invalid_argument &e) {
		RSCTRACE(logger, "Converter already present");
	}

	Factory &factory = rsb::getFactory();

	rsbListenerTransform = factory.createListener(scopeTransforms);
	rsbListenerSync = factory.createListener(scopeSync);
	rsbInformerTransform = factory.createInformer<Transform>(scopeTransforms);
	rsbInformerTransformCollection = factory.createInformer< vector<Transform> >(scopeTransforms);
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
        string usedAuthority = "";
	if (transform.getAuthority() == "") {
		meta.setUserInfo(userKeyAuthority, authority);
                usedAuthority = authority;
	} else {
		meta.setUserInfo(userKeyAuthority, transform.getAuthority());
                usedAuthority = transform.getAuthority();
	}

	boost::mutex::scoped_lock(mutex);
	RSCTRACE(logger,
			"Publishing transform from " << rsbInformerTransform->getId().getIdAsString());
	EventPtr event(rsbInformerTransform->createEvent());
	event->setData(make_shared<Transform>(transform));
	event->setMetaData(meta);

	if (type == STATIC) {
                if(!sendCacheStatic.count(usedAuthority)) {
                    sendCacheStatic[usedAuthority] = map<string, Transform>();
                }
		sendCacheStatic[usedAuthority][cacheKey] = transform;
		event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixStatic)));
	} else if (type == DYNAMIC) {
                if(!sendCacheDynamic.count(usedAuthority)) {
                    sendCacheDynamic[usedAuthority] = map<string, Transform>();
                }
		sendCacheDynamic[usedAuthority][cacheKey] = transform;
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
        if(legacyMode) {
                for(int i=0; i<transforms.size(); i++) {
                        sendTransform(transforms[i], type);
                }
        } else {
            
                Scope scope;
                std::map< std::string, std::map<std::string, Transform> >* cache = NULL;
                if(type == DYNAMIC) {
                    scope = rsbInformerTransformCollection->getScope()->concat(Scope(scopeSuffixDynamic));
                    cache = &sendCacheDynamic;
                } else {
                    scope = rsbInformerTransformCollection->getScope()->concat(Scope(scopeSuffixStatic));
                    cache = &sendCacheStatic;
                }

                if (!rsbInformerTransformCollection) {
                        throw std::runtime_error("communicator was not initialized!");
                }

                RSCTRACE(logger, "sendTransform(vector<>) ");

                map<string, vector<Transform> > transformsByAuthority;

                for(int i=0; i<transforms.size(); i++) {
                	const string cacheKey = transforms[i].getFrameParent() + transforms[i].getFrameChild();
                    string usedAuthority = "";
                    if (transforms[i].getAuthority() == "") {
                            usedAuthority = authority;
                    } else {
                            usedAuthority = transforms[i].getAuthority();
                    }

                    if(!transformsByAuthority.count(usedAuthority)) {
                        transformsByAuthority[usedAuthority] = vector<Transform>();
                    }
                    if(!cache->count(usedAuthority)) {
                        (*cache)[usedAuthority] = map<string, Transform>();
                    }
                    (*cache)[usedAuthority][cacheKey] = transforms[i];
                    transformsByAuthority[usedAuthority].push_back(transforms[i]);
                }

                map<string, vector<Transform> >::iterator authorityIt;
                for(authorityIt = transformsByAuthority.begin(); authorityIt != transformsByAuthority.end(); authorityIt++) {
                    RSCTRACE(logger, "sendTransform(vector<>) Generating event for " << authorityIt->first);

                    const string& authority = authorityIt->first;
                    const vector<Transform>& transforms = authorityIt->second;

                    EventPtr event(rsbInformerTransformCollection->createEvent());

                    MetaData meta;
                    meta.setUserInfo(userKeyAuthority, authority);

                    event->setMetaData(meta);
                    event->setScope(scope);
                    event->setData(make_shared< vector<Transform> >(transforms));
                    
                    rsbInformerTransformCollection->publish(event);
                }
        }
	return true;
}

void TransformCommRsb::publishCache() {
	RSCTRACE(logger, "Publishing cache from " << rsbInformerTransform->getId().getIdAsString());

	map<string, map<string, Transform> >::iterator itAuthorities;

	RSCTRACE(logger, "Publishing dynamic");
        
	for (itAuthorities = sendCacheDynamic.begin(); itAuthorities != sendCacheDynamic.end(); itAuthorities++) {

            RSCTRACE(logger, "Publishing cache for authority " << itAuthorities->first);
            
            map<string, Transform >& authorityCache = itAuthorities->second;
            string authority = itAuthorities->first;

            MetaData meta;
            meta.setUserInfo(userKeyAuthority, authority);
            
            map<string, Transform>::iterator it;
            
            if(legacyMode) {
                
                    for(it = authorityCache.begin(); it != authorityCache.end(); it++) {
                            EventPtr event(rsbInformerTransform->createEvent());
                            event->setData(make_shared< Transform >(it->second));
                            event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixDynamic)));
                            event->setMetaData(meta);
                            rsbInformerTransform->publish(event);
                            RSCTRACE(logger, "Publishing cache key " << it->first);
                    }
                    
            } else {
                    EventPtr event(rsbInformerTransformCollection->createEvent());
                    vector<Transform> transforms;
                    for(it = authorityCache.begin(); it != authorityCache.end(); it++) {
                        transforms.push_back(it->second);
                    }
                    event->setData(make_shared< vector<Transform> >(transforms));
                    event->setScope(rsbInformerTransformCollection->getScope()->concat(Scope(scopeSuffixDynamic)));
                    event->setMetaData(meta);
                    rsbInformerTransformCollection->publish(event);

                    RSCTRACE(logger, "all at once!");
            }
	}

	RSCTRACE(logger, "Publishing static");
        
	for (itAuthorities = sendCacheStatic.begin(); itAuthorities != sendCacheStatic.end(); itAuthorities++) {
            EventPtr event(rsbInformerTransformCollection->createEvent());
            
            map<string, Transform >& authorityCache = itAuthorities->second;
            string authority = itAuthorities->first;

            MetaData meta;
            meta.setUserInfo(userKeyAuthority, authority);
            
            map<string, Transform>::iterator it;
            
            if(legacyMode) {
                
                    for(it = authorityCache.begin(); it != authorityCache.end(); it++) {
                            EventPtr event(rsbInformerTransform->createEvent());
                            event->setData(make_shared< Transform >(it->second));
                            event->setScope(rsbInformerTransform->getScope()->concat(Scope(scopeSuffixStatic)));
                            event->setMetaData(meta);
                            rsbInformerTransform->publish(event);
                            RSCTRACE(logger, "Publishing cache key " << it->first);
                    }
                    
            } else {
                    EventPtr event(rsbInformerTransformCollection->createEvent());
                    vector<Transform> transforms;
                    for(it = authorityCache.begin(); it != authorityCache.end(); it++) {
                        transforms.push_back(it->second);
                    }
                    event->setData(make_shared< vector<Transform> >(transforms));
                    event->setScope(rsbInformerTransformCollection->getScope()->concat(Scope(scopeSuffixStatic)));
                    event->setMetaData(meta);
                    rsbInformerTransformCollection->publish(event);

                    RSCTRACE(logger, "all at once!");
            }
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

template<typename transformPtr>
void processTransform(transformPtr t, string authority, vector<TransformListener::Ptr>& listeners, bool isStatic) {
    t->setAuthority(authority);

    boost::mutex::scoped_lock(mutex);
    vector<TransformListener::Ptr>::iterator it0;
    for (it0 = listeners.begin(); it0 != listeners.end(); ++it0) {
            TransformListener::Ptr l = *it0;
            l->newTransformAvailable(*t, isStatic);
    }
}

void TransformCommRsb::transformCallback(EventPtr event) {
	if (event->getMetaData().getSenderId() == rsbInformerTransform->getId() || event->getMetaData().getSenderId() == rsbInformerTransformCollection->getId()) {
		RSCTRACE(logger,
				"Received transform from myself. Ignore. (id " << event->getMetaData().getSenderId().getIdAsString() << ")");
		return;
	}

	string authority = event->getMetaData().getUserInfo(userKeyAuthority);
        
        bool isStatic = (event->getScope() == rsbInformerTransform->getScope()->concat(Scope(scopeSuffixStatic)));
        
        if(event->getType() == rsc::runtime::typeName<FrameTransformCollection>()) {
            boost::shared_ptr< vector<Transform> > ts = boost::static_pointer_cast< vector<Transform> >(event->getData());
            RSCDEBUG(logger, "Received transform collection from " << authority);
            for(int i=0; i<ts->size(); i++) {
                Transform* t = &(ts->at(i));
                
                RSCTRACE(logger, "Received transform: " << *t);
                
                processTransform(t, authority, listeners, isStatic);
            }
        } else if(event->getType() == rsc::runtime::typeName<FrameTransform>()) {
            boost::shared_ptr<Transform> t = boost::static_pointer_cast<Transform>(event->getData());
            
            RSCDEBUG(logger, "Received transform from " << authority);
            RSCTRACE(logger, "Received transform: " << *t);
            processTransform(t, authority, listeners, isStatic);
        } else {
            RSCWARN(logger, "Received invalid data type: " << event->getType() << ", expected " << rsc::runtime::typeName<FrameTransformCollection>() << " or " << rsc::runtime::typeName<FrameTransform>());
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
