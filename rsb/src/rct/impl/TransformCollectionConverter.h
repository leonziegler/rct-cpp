/*
 * TransformConverter.h
 *
 *  Created on: Mar 28, 2015
 *      Author: lziegler
 */

#pragma once


#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <rsb/converter/Converter.h>

#include "TransformConverter.h"

#include <rct/Transform.h>
#include <rct/FrameTransformCollection.pb.h>

namespace rct {

class TransformCollectionConverter: public rsb::converter::Converter<std::string>  {
public:
	typedef boost::shared_ptr<TransformCollectionConverter> Ptr;

	TransformCollectionConverter();
        
        inline void rstToDomain(const FrameTransformCollection &ts, std::vector<Transform>& transforms);
        inline void domainToRST(const std::vector<Transform>& transforms, FrameTransformCollection &ts);
        
	virtual ~TransformCollectionConverter();

    std::string getWireSchema() const;

    std::string serialize(const rsb::AnnotatedData &data, std::string &wire);
    rsb::AnnotatedData deserialize(const std::string &wireType,
            const std::string &wire);

private:
    TransformConverter transformConverter;
    boost::shared_ptr<rsb::converter::Converter<std::string> > converter;
};

} /* namespace rct */
