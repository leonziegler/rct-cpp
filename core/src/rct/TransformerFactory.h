/*
 * TransformerFactory.h
 *
 *  Created on: Dec 7, 2014
 *      Author: leon
 */

#pragma once

#include "Transformer.h"
#include <rsc/patterns/Singleton.h>
#include <exception>

namespace rct {

class TransformerFactory;

TransformerFactory& getTransformerFactory();

class TransformerFactoryException: public std::exception {
public:
	TransformerFactoryException(const std::string &msg) : msg(msg) {
	}
	virtual ~TransformerFactoryException() {
	}
	virtual const char* what() const {
		return msg.c_str();
	}
private:
	std::string msg;
};

class TransformerFactory: private rsc::patterns::Singleton<TransformerFactory> {
public:
	virtual ~TransformerFactory();

	Transformer::Ptr createTransformer(const boost::posix_time::time_duration& cacheTime) const;

private:
	TransformerFactory();
	friend TransformerFactory& getTransformerFactory();
	static TransformerFactory& getInstanceBase();
};

}  // namespace rct
