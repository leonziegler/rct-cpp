/*
 * TransformCollectionConverter.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: lziegler
 */

#include "TransformCollectionConverter.h"
#include <rct/Transform.h>

#include <rsb/converter/SerializationException.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <rct/FrameTransformCollection.pb.h>
#include <rst/geometry/Pose.pb.h>

using namespace std;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;

namespace rct {

void TransformCollectionConverter::domainToRST(const vector<Transform>& transforms, FrameTransformCollection &ts) {
    for(int i=0; i<transforms.size(); ++i) {
        FrameTransform* t = ts.add_transforms();
        
	transformConverter.domainToRST(transforms[i], *t);
    }
}
void TransformCollectionConverter::rstToDomain(const FrameTransformCollection &ts, vector<Transform>& transforms) {
    
    for(int i=0; i<ts.transforms_size(); ++i) {
        Transform transform;
        
	transformConverter.rstToDomain(ts.transforms(i), transform);
        
        transforms.push_back(transform);
    }
}

TransformCollectionConverter::TransformCollectionConverter():
                rsb::converter::Converter<string>(
                        rsc::runtime::typeName<FrameTransformCollection>(),
                        RSB_TYPE_TAG(vector<Transform>)) {
	converter = shared_ptr<Converter<string> >(new ProtocolBufferConverter<FrameTransformCollection>);
}

TransformCollectionConverter::~TransformCollectionConverter() {
}

std::string TransformCollectionConverter::getWireSchema() const {
	return converter->getWireSchema();
}

std::string TransformCollectionConverter::serialize(const rsb::AnnotatedData& data, std::string& wire) {
	// Cast to original domain type
	shared_ptr< vector<Transform> > domain = static_pointer_cast< vector<Transform> >(data.second);

	// Fill protocol buffer object
	rct::FrameTransformCollection proto;

	//
	domainToRST(*domain, proto);

	// Use embedded ProtoBuf converter for serialization to wire
	return converter->serialize(
			make_pair(rsc::runtime::typeName<rct::FrameTransformCollection>(),
					boost::shared_ptr<void>(&proto, rsc::misc::NullDeleter())), wire);
}

rsb::AnnotatedData TransformCollectionConverter::deserialize(const std::string& wireType,
		const std::string& wire) {

	// Deserialize and cast to specific ProtoBuf type
	boost::shared_ptr<rct::FrameTransformCollection> proto = boost::static_pointer_cast<rct::FrameTransformCollection>(
			converter->deserialize(wireType, wire).second);

	// Instantiate domain object
	boost::shared_ptr< vector<Transform> > domain(new vector<Transform>());

	// Read domain data from ProtoBuf
	rstToDomain(*proto, *domain);

	return rsb::AnnotatedData(getDataType(), domain);
}

} /* namespace rct */
