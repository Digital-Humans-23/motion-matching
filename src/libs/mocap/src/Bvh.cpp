//
// Created by Dongho Kang on 04.12.21.
//

#include "mocap/bvh/Bvh.h"

namespace crl::mocap {

Bvh::Bvh() : num_frames_(0), frame_time_(0), num_channels_(0), name_("") {}

Bvh::Bvh(const std::string &name) : num_frames_(0), frame_time_(0), num_channels_(0), name_(name) {}

void Bvh::addJoint(const std::shared_ptr<BvhJoint> &joint) {
    joints_.push_back(joint);
    num_channels_ += joint->getNumberOfChannels();
}

const std::shared_ptr<BvhJoint> Bvh::getRootJoint() const {
    return root_joint_;
}

std::vector<std::shared_ptr<BvhJoint>> Bvh::getJoints() const {
    return joints_;
}

unsigned Bvh::getFramesCount() const {
    return num_frames_;
}

double Bvh::getFrameTime() const {
    return frame_time_;
}

size_t Bvh::getTotalNumberOfChannels() const {
    return num_channels_;
}

void Bvh::setRootJoint(const std::shared_ptr<BvhJoint> &arg) {
    root_joint_ = arg;
}

void Bvh::setJoints(const std::vector<std::shared_ptr<BvhJoint>> &arg) {
    joints_ = arg;
}

}  // namespace crl::mocap