/*
 * TransformerConfig.h
 *
 *  Created on: Dec 20, 2014
 *      Author: leon
 */

#pragma once

#include <rsc/config/OptionHandler.h>
#include <rsc/runtime/Printable.h>
#include <rsc/runtime/Properties.h>
#include <boost/format.hpp>

namespace rct {

class TransformerConfig: public rsc::config::OptionHandler,
		public rsc::runtime::Printable {
public:

	enum CommunicatorType {
		AUTO, RSB, ROS
	};

	enum CoreType {
	    DEFAULT, TF2
	};

	static std::string typeToString(CommunicatorType type) {
		switch (type) {
		case AUTO:
			return "AUTO";
		case RSB:
			return "RSB";
		case ROS:
			return "ROS";
		default:
			return "UNKNOWN";
		}
	}

    static std::string typeToString(CoreType type) {
        switch (type) {
        case TF2:
            return "TF2";
        default:
            return "DEFAULT";
        }
    }

	TransformerConfig() :
			commType(AUTO), coreType(DEFAULT), cacheTime(
					boost::posix_time::time_duration(0, 0, 30)) {
	}
	virtual ~TransformerConfig() {
	}

	const boost::posix_time::time_duration& getCacheTime() const {
		return cacheTime;
	}

	void setCacheTime(const boost::posix_time::time_duration& cacheTime) {
		this->cacheTime = cacheTime;
	}

	CommunicatorType getCommType() const {
		return commType;
	}

	void setCommType(CommunicatorType commType) {
		this->commType = commType;
	}

    CoreType getCoreType() const {
        return coreType;
    }

    void setCoreType(CoreType coreType) {
        this->coreType = coreType;
    }

	/**
	 * Returns additional options besides the transformer-specific ones.
	 *
	 * @return copy of additional options
	 */
	rsc::runtime::Properties getOptions() const {
		return options;
	}

	/**
	 * Returns a mutable reference to the freestyle options in this
	 * configuration.
	 *
	 * @return mutable reference to additional options
	 */
	rsc::runtime::Properties& mutableOptions() {
		return options;
	}

	/**
	 * Sets the additional options besides the transformer-specific ones.
	 *
	 * @param options new options replacing all old ones
	 */
	void setOptions(const rsc::runtime::Properties& options) {
		this->options = options;
	}

	void printContents(std::ostream& stream) const {
		switch (commType) {
		case AUTO:
			stream << "comm = AUTO";
			break;
		case RSB:
			stream << "comm = RSB";
			break;
		case ROS:
			stream << "comm = ROS";
			break;
		default:
			stream << "comm = UNKNOWN";
			break;
		}
		stream << ", ";
		switch (coreType) {
        case TF2:
            stream << "core = TF2";
            break;
        default:
            stream << "core = DEFAULT";
            break;
        }
		stream << ", cacheTime = " << cacheTime;

	}

private:
	CommunicatorType commType;
	CoreType coreType;
	boost::posix_time::time_duration cacheTime;
	rsc::runtime::Properties options;
	void handleOption(const std::vector<std::string>& key,
			const std::string& value) {

		if (key[0] == "core") {
			if (key.size() != 2) {
				throw std::invalid_argument(
						boost::str(
								boost::format(
										"Option key `%1%' has invalid number of components; options related to core have to have two components.")
										% key));
			}

			if (key[1] == "cachetime") {
				this->cacheTime = boost::posix_time::duration_from_string(
						value);
			}
		} else if (key[0] == "communicator") {
			if (key.size() != 2) {
				throw std::invalid_argument(
						boost::str(
								boost::format(
										"Option key `%1%' has invalid number of components; options related to communication have to have two components.")
										% key));
			}
			if (key[1] == "type") {
				if (value == "AUTO") {
					this->commType = AUTO;
				} else if (value == "RSB") {
					this->commType = RSB;
				} else if (value == "ROS") {
					this->commType = ROS;
				} else {
					throw std::invalid_argument(
							boost::str(
									boost::format(
											"Value `%1%' does not name a communicator type.")
											% value));
				}
			}

        } else if (key[0] == "core") {
            if (key.size() != 2) {
                throw std::invalid_argument(
                        boost::str(
                                boost::format(
                                        "Option key `%1%' has invalid number of components; options related to core implementation have to have two components.")
                                        % key));
            }
            if (key[1] == "type") {
                if (value == "TF2") {
                    this->coreType = TF2;
                } else if (value == "DEFAULT") {
                    this->coreType = DEFAULT;
                } else {
                    throw std::invalid_argument(
                            boost::str(
                                    boost::format(
                                            "Value `%1%' does not name a core type.")
                                            % value));
                }
            }

		} else {
			if (key.size() == 1) {
				this->options[key[0]] = value;
			}
		}
	}
};

}
