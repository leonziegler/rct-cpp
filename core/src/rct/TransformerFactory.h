/*
 * TransformerFactory.h
 *
 *  Created on: Dec 7, 2014
 *      Author: leon
 */

#pragma once

#include "impl/TransformerCore.h"
#include "Transformer.h"
#include "TransformerConfig.h"
#include <rsc/patterns/Singleton.h>
#include <exception>

namespace rct {

class TransformerFactory;

TransformerFactory& getTransformerFactory();

class TransformerFactoryException: public std::exception {
public:
	TransformerFactoryException(const std::string &msg) : msg(msg) {
	}
	virtual ~TransformerFactoryException() throw() {
	}
	virtual const char* what() const throw() {
		return msg.c_str();
	}
private:
	std::string msg;
};

class TransformerFactory: private rsc::patterns::Singleton<TransformerFactory> {
public:
	virtual ~TransformerFactory();

	Transformer::Ptr createTransformer(const std::string &name, const TransformerConfig& cacheTime = TransformerConfig()) const;
	Transformer::Ptr createTransformer(const std::string &name, const TransformListener::Ptr& listener, const TransformerConfig& cacheTime = TransformerConfig()) const;
	Transformer::Ptr createTransformer(const std::string &name, const std::vector<TransformListener::Ptr>& listeners, const TransformerConfig& cacheTime = TransformerConfig()) const;

	friend class rsc::patterns::Singleton<TransformerFactory>;
private:
	TransformerFactory();
	static TransformerFactory& getInstanceBase();
	friend TransformerFactory& getTransformerFactory();
};

}  // namespace rct
