/*
 * TransformCommRsb.h
 *
 *  Created on: Dec 15, 2014
 *      Author: leon
 */

#pragma once

#include <rct/impl/TransformCommunicator.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <rsb/Listener.h>
#include <rsb/Informer.h>
#include <boost/shared_ptr.hpp>
#include <log4cxx/logger.h>

#include "FrameTransform.pb.h"

namespace rct {

class TransformCommRsb: public TransformCommunicator {
public:
	typedef boost::shared_ptr<TransformCommRsb> Ptr;
	TransformCommRsb(const std::string &authority, const boost::posix_time::time_duration& cacheTime, const TransformListener::Ptr& listener);
	TransformCommRsb(const std::string &authority, const boost::posix_time::time_duration& cacheTime, const std::vector<TransformListener::Ptr>& listeners);
	virtual ~TransformCommRsb();

	virtual void init(const TransformerConfig &conf);
	virtual void requestSync();

	/** \brief Add transform information to the rct data structure
	 * \param transform The transform to store
	 * \param authority The source of the information for this transform
	 * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
	 * \return True unless an error occured
	 */
	virtual bool sendTransform(const Transform& transform, TransformType type);
	virtual bool sendTransform(const std::vector<Transform>& transforms, TransformType type);

	virtual void addTransformListener(const TransformListener::Ptr& listener);
	virtual void addTransformListener(const std::vector<TransformListener::Ptr>& listeners);
	virtual void removeTransformListener(const TransformListener::Ptr& listener);

	static void convertTransformToPb(const Transform& transform, boost::shared_ptr<FrameTransform> &t);
	static void convertPbToTransform(const boost::shared_ptr<FrameTransform> &t, Transform& transform);

	void printContents(std::ostream& stream) const;

	virtual std::string getAuthorityName() const;

private:
	rsb::ListenerPtr rsbListenerTransform;
	rsb::Informer<FrameTransform>::Ptr rsbInformerTransform;
	rsb::ListenerPtr rsbListenerTrigger;
	rsb::Informer<void>::Ptr rsbInformerTrigger;
	std::vector<TransformListener::Ptr> listeners;
	boost::mutex mutex;
	std::map<std::string, std::pair<boost::shared_ptr<FrameTransform>, rsb::MetaData> > sendCacheDynamic;
	std::map<std::string, std::pair<boost::shared_ptr<FrameTransform>, rsb::MetaData> > sendCacheStatic;
	std::string authority;

	void frameTransformCallback(rsb::EventPtr t);
	void triggerCallback(rsb::EventPtr t);
	void publishCache();

	static log4cxx::LoggerPtr logger;
};
}  // namespace rct
