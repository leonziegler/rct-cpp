/*
 * TransformConverter.h
 *
 *  Created on: Mar 28, 2015
 *      Author: lziegler
 */

#pragma once


#include <string>
#include <boost/shared_ptr.hpp>

#include <rsb/converter/Converter.h>

#include <rct/Transform.h>
#include <rct/FrameTransform.pb.h>

namespace rct {

class TransformConverter: public rsb::converter::Converter<std::string>  {
public:
	typedef boost::shared_ptr<TransformConverter> Ptr;
        
        void domainToRST(const Transform& transform, FrameTransform &t);
        
        void rstToDomain(const FrameTransform &t, Transform& transform);

	TransformConverter();
	virtual ~TransformConverter();

    std::string getWireSchema() const;

    std::string serialize(const rsb::AnnotatedData &data, std::string &wire);
    rsb::AnnotatedData deserialize(const std::string &wireType,
            const std::string &wire);

private:

    boost::shared_ptr<rsb::converter::Converter<std::string> > converter;
};

} /* namespace rct */
