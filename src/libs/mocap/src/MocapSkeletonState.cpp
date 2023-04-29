#include "mocap/MocapSkeletonState.h"

namespace crl::mocap {

MocapSkeletonState::MocapSkeletonState(MocapSkeleton *skeleton, bool useDefaultAngles) {
    skeleton->populateState(this, useDefaultAngles);
}

MocapSkeletonState::MocapSkeletonState(const MocapSkeletonState &other) {
    this->rootQ = other.rootQ;
    this->rootPos = other.rootPos;
    this->rootVel = other.rootVel;
    this->rootAngVel = other.rootAngVel;
    this->jointStates = other.jointStates;
}

void MocapSkeletonState::setJointCount(int jCount) {
    jointStates.resize(jCount);
}

void MocapSkeletonState::setRootPosition(P3D p) {
    rootPos = p;
}

void MocapSkeletonState::setRootOrientation(Quaternion q) {
    rootQ = q;
}

void MocapSkeletonState::setRootVelocity(V3D v) {
    rootVel = v;
}

void MocapSkeletonState::setRootAngularVelocity(V3D w) {
    rootAngVel = w;
}

void MocapSkeletonState::setJointRelativeOrientation(const Quaternion &q, int jIdx) {
    if ((uint)jIdx < jointStates.size())
        jointStates[jIdx].qRel = q;
}

void MocapSkeletonState::setJointRelativeAngVelocity(V3D w, int jIdx) {
    if ((uint)jIdx < jointStates.size())
        jointStates[jIdx].angVelRel = w;
}

void MocapSkeletonState::setJointTranslation(const V3D &t, int jIdx) {
    if ((uint)jIdx < jointStates.size())
        jointStates[jIdx].tRel = t;
}

void MocapSkeletonState::setJointRelativeVelocity(V3D v, int jIdx) {
    if ((uint)jIdx < jointStates.size())
        jointStates[jIdx].velRel = v;
}

int MocapSkeletonState::getJointCount() const {
    return jointStates.size();
}

P3D MocapSkeletonState::getRootPosition() const {
    return rootPos;
}

Quaternion MocapSkeletonState::getRootOrientation() const {
    return rootQ;
}

V3D MocapSkeletonState::getRootVelocity() const {
    return rootVel;
}

V3D MocapSkeletonState::getRootAngularVelocity() const {
    return rootAngVel;
}

Quaternion MocapSkeletonState::getJointRelativeOrientation(int jIdx) const {
    if ((uint)jIdx < jointStates.size())
        return jointStates[jIdx].qRel;
    return Quaternion::Identity();
}

V3D MocapSkeletonState::getJointRelativeAngVelocity(int jIdx) const {
    if ((uint)jIdx < jointStates.size())
        return jointStates[jIdx].angVelRel;
    return V3D();
}

V3D MocapSkeletonState::getJointTranslation(int jIdx) const {
    if ((uint)jIdx < jointStates.size())
        return jointStates[jIdx].tRel;
    return V3D();
}

V3D MocapSkeletonState::getJointRelativeVelocity(int jIdx) const {
    if ((uint)jIdx < jointStates.size())
        return jointStates[jIdx].velRel;
    return V3D();
}

}  // namespace crl::mocap