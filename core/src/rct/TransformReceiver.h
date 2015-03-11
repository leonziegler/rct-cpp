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

class TransformReceiver: public virtual rsc::runtime::Printable, public boost::noncopyable {
public:
	typedef boost::shared_ptr<TransformReceiver> Ptr;
	typedef rsc::threading::Future<Transform> FutureType;
	typedef boost::shared_ptr<FutureType> FuturePtr;

	TransformReceiver(const TransformerCore::Ptr &core, const TransformCommunicator::Ptr &comm,
			const TransformerConfig &conf = TransformerConfig());
	virtual ~TransformReceiver();

	/** \brief Get the transform between two frames by frame ID.
	 * \param target_frame The frame to which data should be transformed
	 * \param source_frame The frame where the data originated
	 * \param time The time at which the value of the transform is desired. (0 will get the latest)
	 * \return The transform between the frames
	 *
	 */
	virtual Transform lookupTransform(const std::string& target_frame,
			const std::string& source_frame, const boost::posix_time::ptime& time) const;

	/** \brief Get the transform between two frames by frame ID assuming fixed frame.
	 * \param target_frame The frame to which data should be transformed
	 * \param target_time The time to which the data should be transformed. (0 will get the latest)
	 * \param source_frame The frame where the data originated
	 * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
	 * \param fixed_frame The frame in which to assume the transform is constant in time.
	 * \return The transform between the frames
	 */
	virtual Transform lookupTransform(const std::string& target_frame,
			const boost::posix_time::ptime& target_time, const std::string& source_frame,
			const boost::posix_time::ptime& source_time, const std::string& fixed_frame) const;

	/** \brief Request the transform between two frames by frame ID.
	 * \param target_frame The frame to which data should be transformed
	 * \param source_frame The frame where the data originated
	 * \param time The time at which the value of the transform is desired. (0 will get the latest)
	 * \return A future object representing the request status and transform between the frames
	 *
	 */
	virtual FuturePtr requestTransform(const std::string& target_frame,
			const std::string& source_frame, const boost::posix_time::ptime& time);

	/** \brief Test if a transform is possible
	 * \param target_frame The frame into which to transform
	 * \param source_frame The frame from which to transform
	 * \param time The time at which to transform
	 * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
	 * \return True if the transform is possible, false otherwise
	 */
	virtual bool canTransform(const std::string& target_frame, const std::string& source_frame,
			const boost::posix_time::ptime& time, std::string* error_msg = NULL) const;

	/** \brief Test if a transform is possible
	 * \param target_frame The frame into which to transform
	 * \param target_time The time into which to transform
	 * \param source_frame The frame from which to transform
	 * \param source_time The time from which to transform
	 * \param fixed_frame The frame in which to treat the transform as constant in time
	 * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
	 * \return True if the transform is possible, false otherwise
	 */
	virtual bool canTransform(const std::string& target_frame,
			const boost::posix_time::ptime &target_time, const std::string& source_frame,
			const boost::posix_time::ptime &source_time, const std::string& fixed_frame,
			std::string* error_msg = NULL) const;

	TransformerCore::ConstPtr getCore() const;

	void printContents(std::ostream& stream) const;
	TransformerConfig getConfig() const;
	std::string getAuthorityName() const;
	void shutdown();
private:

	TransformCommunicator::Ptr comm;
	TransformerCore::Ptr core;
	TransformerConfig config;
};

} /* namespace rct */
