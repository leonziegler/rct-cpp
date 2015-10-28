/*
 * FrameTreeSimple.h
 *
 *  Created on: Oct 19, 2015
 *      Author: leon
 */

#pragma once

#include <boost/date_time.hpp>
#include <boost/signals2.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>

#include <rsc/logging/Logger.h>

#include "FrameHistory.h"
#include "../Transform.h"

namespace rct {

class FrameTreeSimple {
public:
    static const uint32_t MAX_GRAPH_DEPTH = 1000UL;

    FrameTreeSimple(const boost::posix_time::time_duration& cacheTime);
    virtual ~FrameTreeSimple();

    /** \brief Clear all data */
    virtual void clear();

    /** \brief Add transform information to the rct data structure
     * \param transform The transform to store
     * \param authority The source of the information for this transform
     * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
     * \return True unless an error occured
     */
    virtual bool setTransform(const Transform& transform, bool is_static = false);

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
     *
     * Possible exceptions LookupException, ConnectivityException,
     * ExtrapolationException, InvalidArgumentException
     */

    virtual Transform lookupTransform(const std::string& target_frame,
            const boost::posix_time::ptime &target_time, const std::string& source_frame,
            const boost::posix_time::ptime &source_time, const std::string& fixed_frame) const;

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param source_frame The frame from which to transform
     * \param time The time at which to transform
     * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
     * \return True if the transform is possible, false otherwise
     */
    virtual bool canTransform(const std::string& target_frame, const std::string& source_frame,
            const boost::posix_time::ptime &time) const;

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
            const boost::posix_time::ptime &source_time, const std::string& fixed_frame) const;

    /** \brief A way to get a std::vector of available frame ids */
    virtual std::vector<std::string> getFrameStrings() const;

    /**@brief Check if a frame exists in the tree
     * @param frame_id_str The frame id in question */
    virtual bool frameExists(const std::string& frame_id_str) const;

    /**@brief Fill the parent of a frame.
     * @param frame_id The frame id of the frame in question
     * @param parent The reference to the string to fill the parent
     * Returns true unless "NO_PARENT" */
    virtual std::string getParent(const std::string& frame_id,
            const boost::posix_time::ptime &time) const;

    /** \brief Backwards compatabilityA way to see what frames have been cached
     * Useful for debugging
     */
    virtual std::string allFramesAsDot() const;

    /** \brief A way to see what frames have been cached in yaml format
     * Useful for debugging tools
     */
    virtual std::string allFramesAsYAML() const;

    /** \brief A way to see what frames have been cached
     * Useful for debugging
     */
    virtual std::string allFramesAsString() const;

    boost::signals2::connection addTransformsChangedListener(boost::function<void(void)> callback);
    void removeTransformsChangedListener(boost::signals2::connection c);

private:

    boost::posix_time::time_duration cacheTime;

    std::vector<FrameHistoryPtr> frames;

    /** \brief A mutex to protect testing and allocating new frames on the above vector. */
    mutable boost::mutex frameMutex;
    mutable boost::mutex transformable_requests_mutex_;

    boost::unordered_map<std::string, uint32_t> frameIDs;
    std::vector<std::string> frameIDsReverse;
    std::map<uint32_t, std::string> authorities;

    typedef boost::signals2::signal<void(void)> TransformsChangedSignal;
    TransformsChangedSignal _transforms_changed_;

    static const rsc::logging::LoggerPtr logger;

    uint32_t lookupOrInsertFrameNumber(const std::string& frameStr);
    void testTransformableRequests();
    FrameHistoryPtr getFrame(uint32_t frameId) const;
    FrameHistoryPtr allocateFrame(uint32_t cfid, bool isStatic);

    uint32_t lookupFrameNumber(const std::string& frameStr) const;

    const std::string& lookupFrameString(uint32_t frameNum) const;

    uint32_t validateFrameId(const std::string& function, const std::string& frame_id) const;
    bool warnFrameId(const std::string& function, const std::string& frame_id) const;

    boost::posix_time::ptime getLatestCommonTime(uint32_t target_frame,
            uint32_t source_frame) const;

    std::string allFramesAsStringNoLock() const;
    bool canTransformNoLock(uint32_t target_id, uint32_t source_id,
            const boost::posix_time::ptime& time) const;
    bool canTransformInternal(uint32_t target_id, uint32_t source_id,
            const boost::posix_time::ptime& time) const;

    template<typename F>
    void walkToTopParent(F& f, boost::posix_time::ptime time, uint32_t target_id,
            uint32_t source_id, std::vector<uint32_t> *frame_chain) const;

};

} /* namespace rct */
