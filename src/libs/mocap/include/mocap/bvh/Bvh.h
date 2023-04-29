#pragma once

// Lib includes
#include "mocap/bvh/BvhJoint.h"

// STL includes
#include <memory>
#include <vector>

namespace crl::mocap {

/** Class created for storing motion data from bvh file */
class Bvh {
public:
    /** Constructor of Bvh object
     *  @details  Initializes local variables
     */
    Bvh();

    /** Overloaded constrcutor with name
     *  @details  Initialized locals and sets the animation name
     */
    explicit Bvh(const std::string &name);

    /** Adds joint to Bvh object
     *  @details  Adds joint and increases number of data channels
     *  @param  joint  The joint that will be added
     */
    void addJoint(const std::shared_ptr<BvhJoint> &joint);

    /** Gets the root joint
     *  @return  The root joint
     */
    const std::shared_ptr<BvhJoint> getRootJoint() const;

    /** Gets all joints
     *  @return  The all joints
     */
    std::vector<std::shared_ptr<BvhJoint>> getJoints() const;

    /** Gets the number of data frames
     *  @return  The number of frames
     */
    unsigned getFramesCount() const;

    /** Gets the frame time
     *  @return  The single frame time (in second)
     */
    double getFrameTime() const;

    /** Gets the total number of channels
     *  @return  The number of data channels
     */
    size_t getTotalNumberOfChannels() const;

    /** Sets the root joint
     *  @param  arg  The root joint to be set
     */
    void setRootJoint(const std::shared_ptr<BvhJoint> &arg);

    /** Sets the all joint at once
     *  @param  arg  The all joints to be set
     */
    void setJoints(const std::vector<std::shared_ptr<BvhJoint>> &arg);

    /** Sets the number of data frames
     *  @param  arg  The number of frames to be set
     */
    void setNumberOfFrames(const unsigned arg) {
        num_frames_ = arg;
    }

    /** Sets the single data frame time
     *  @param  arg  The time of frame to be set
     */
    void setFrameTime(const double arg) {
        frame_time_ = arg;
    }

    /** Sets the name of the mocap **/
    void setName(const std::string &name) {
        name_ = name;
    }

    /** Get the name of the animation **/
    std::string getName() const {
        return name_;
    }

    bool operator<(const Bvh &other) const {
        return getName() < other.getName();
    }

private:
    /** A root joint in this bvh file */
    std::shared_ptr<BvhJoint> root_joint_;
    /** All joints in file in order of parse */
    std::vector<std::shared_ptr<BvhJoint>> joints_;
    /** A number of motion frames in this bvh file */
    unsigned num_frames_;
    /** A time of single frame */
    double frame_time_;
    /** Number of channels of all joints */
    size_t num_channels_;
    /** The name of the file **/
    std::string name_;
};

}  // namespace crl::mocap
