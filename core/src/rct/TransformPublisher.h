/*
 * Transformer.h
 *
 *  Created on: 05.12.2014
 *      Author: leon
 */

#pragma once

#include "Transform.h"
#include "TransformType.h"
#include "TransformerConfig.h"
#include "impl/TransformCommunicator.h"
#include "impl/TransformerCore.h"
#include <Eigen/Geometry>
#include <string>
#include <boost/integer.hpp>
#include <boost/thread.hpp>

namespace rct {

class TransformPublisher: public virtual rsc::runtime::Printable,
		public boost::noncopyable {
public:
	typedef boost::shared_ptr<TransformPublisher> Ptr;

	TransformPublisher(const TransformCommunicator::Ptr &comm, const TransformerConfig &conf = TransformerConfig());
	virtual ~TransformPublisher();

	/** \brief Add transform information to the rct data structure
	 * \param transform The transform to store
	 * \param authority The source of the information for this transform
	 * \param type Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
	 * \return True unless an error occured
	 */
	virtual bool sendTransform(const Transform& transform, TransformType type);

	/** \brief Add transform information to the rct data structure
	 * \param transform The transform to store
	 * \param authority The source of the information for this transform
	 * \param type Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
	 * \return True unless an error occured
	 */
	virtual bool sendTransform(const std::vector<Transform>& transforms, TransformType type);

	void printContents(std::ostream& stream) const;
	TransformerConfig getConfig() const;
	std::string getAuthorityName() const;
	void shutdown();
private:

	TransformCommunicator::Ptr comm;
	TransformerConfig config;
};

} /* namespace rct */
