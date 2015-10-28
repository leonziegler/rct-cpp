/*
 * FrameHistory.h
 *
 *  Created on: Oct 20, 2015
 *      Author: leon
 */

#pragma once

#include <boost/date_time.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Geometry>

#include <list>

namespace rct {

struct TransformStorage {
    Eigen::Affine3d transform;
    uint32_t frameParent;
    uint32_t frameChild;
    boost::posix_time::ptime time;

    TransformStorage() :
            frameParent(-1), frameChild(-1) {
    }
    TransformStorage(const TransformStorage& other) :
                    transform(other.transform),
                    frameChild(other.frameChild),
                    frameParent(other.frameParent),
                    time(other.time) {
    }
};

class FrameHistory {
public:
    FrameHistory(){}
    virtual ~FrameHistory(){}

    /** \brief Access data from the history at a specific time */
    virtual TransformStorage getData(boost::posix_time::ptime time)=0;

    /** \brief Insert data into the history */
    virtual bool insertData(const TransformStorage& new_data)=0;

    /** @brief Clear the list of stored values */
    virtual void clearList()=0;

    /** \brief Retrieve the parent at a specific time */
    virtual uint32_t getParent(boost::posix_time::ptime time) = 0;

    /**
     * \brief Get the latest time stored in this cache, and the parent associated with it.
     */
    virtual std::pair<boost::posix_time::ptime, uint32_t> getLatestTimeAndParent() = 0;

    /** @brief Get the length of the stored list */
    virtual unsigned int getListLength()=0;

    /** @brief Get the latest time cached */
    virtual boost::posix_time::ptime getLatestTime()=0;

    /** @brief Get the oldest time cached */
    virtual boost::posix_time::ptime getOldestTime()=0;
};

typedef boost::shared_ptr<FrameHistory> FrameHistoryPtr;

class DynamicFrameHistory: public FrameHistory {
public:
    static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
    static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
    static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

    DynamicFrameHistory(const boost::posix_time::time_duration& maxStorageTime);
    virtual ~DynamicFrameHistory();

    TransformStorage getData(boost::posix_time::ptime time);

    bool insertData(const TransformStorage& new_data);

    void clearList();

    uint32_t getParent(boost::posix_time::ptime time);

    std::pair<boost::posix_time::ptime, uint32_t> getLatestTimeAndParent();
    unsigned int getListLength();
    boost::posix_time::ptime getLatestTime();
    boost::posix_time::ptime getOldestTime();

private:
    std::list<TransformStorage> storage;

    boost::posix_time::time_duration maxStorageTime;

    inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two,
            boost::posix_time::ptime target_time);

    inline TransformStorage interpolate(const TransformStorage& one, const TransformStorage& two,
            boost::posix_time::ptime time);

    void pruneList();
};

class StaticFrameHistory: public FrameHistory {
    TransformStorage getData(boost::posix_time::ptime time);

    bool insertData(const TransformStorage& new_data);

    void clearList();

    uint32_t getParent(boost::posix_time::ptime time);

    std::pair<boost::posix_time::ptime, uint32_t> getLatestTimeAndParent();
    unsigned int getListLength();
    boost::posix_time::ptime getLatestTime();
    boost::posix_time::ptime getOldestTime();
private:
    TransformStorage storage;
};

} /* namespace rct */
