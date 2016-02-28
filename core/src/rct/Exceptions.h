/*
 * Exceptions.h
 *
 *  Created on: Oct 19, 2015
 *      Author: leon
 */

#pragma once

#include <exception>

namespace rct {

/**
 * \brief General exception for the rct library.
 */
class RctException: public std::exception {
public:
    RctException(const std::string &msg) throw() :
            msg(msg), hasReason(false) {

    }
    RctException(const std::string &msg, const std::exception &reason) throw() :
            msg(msg), reason(reason), hasReason(true) {

    }
    RctException(const std::exception &reason) throw() :
            reason(reason), hasReason(true) {

    }
    virtual ~RctException() throw () {
    }
    virtual const char* what() const throw () {
        if (msg.empty())
            return "RctException";
        else if (hasReason)
            return (msg + ". Reason: " + reason.what()).c_str();
        else
            return msg.c_str();
    }

private:
    std::string msg;
    std::exception reason;
    bool hasReason;
};

/**
 * \brief Exception for errors related to impossible extrapolation of transformations into the
 * future or past.
 *
 * This usually occurs when lookups for some point in time are requested that would require
 * extrapolation beyond current limits.
 */
class ExtrapolationException: public RctException {
public:
    ExtrapolationException(const std::string &msg) throw() :
            RctException(msg) {

    }
    ExtrapolationException(const std::string &msg, const std::exception &reason) throw() :
            RctException(msg, reason) {

    }
    ExtrapolationException(const std::exception &reason) throw() :
            RctException("ExtrapolationException", reason) {

    }
    virtual ~ExtrapolationException() throw () {
    }
};

/**
 * \brief Exception for errors related to impossible transformation lookup.
 */
class LookupException: public RctException {
public:
    LookupException(const std::string &msg) throw() :
            RctException(msg) {

    }
    LookupException(const std::string &msg, const std::exception &reason) throw() :
            RctException(msg, reason) {

    }
    LookupException(const std::exception &reason) throw() :
            RctException("LookupException", reason) {

    }
    virtual ~LookupException() throw () {
    }
};

/**
 * \brief Exception for errors related to wrong arguments.
 */
class InvalidArgumentException: public RctException {
public:
    InvalidArgumentException(const std::string &msg) throw() :
            RctException(msg) {

    }
    InvalidArgumentException(const std::string &msg, const std::exception &reason) throw() :
            RctException(msg, reason) {

    }
    InvalidArgumentException(const std::exception &reason) throw() :
            RctException("InvalidArgumentException", reason) {

    }
    virtual ~InvalidArgumentException() throw () {
    }
};

/**
 * \brief Exception for errors related to unconnected transformation trees.
 */
class ConnectivityException: public RctException {
public:
    ConnectivityException(const std::string &msg) throw() :
            RctException(msg) {

    }
    ConnectivityException(const std::string &msg, const std::exception &reason) throw() :
            RctException(msg, reason) {

    }
    ConnectivityException(const std::exception &reason) throw() :
            RctException("ConnectivityException", reason) {

    }
    virtual ~ConnectivityException() throw () {
    }
};

}

