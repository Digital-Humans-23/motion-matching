#pragma once

#include "mocap/MocapSkeleton.h"

namespace crl::mocap {

struct JointState {
    // relative orientation from parent in parent's frame
    Quaternion qRel = Quaternion::Identity();
    // translation
    // note. that origin of this joint's frame w.r.t. parent's frame = offset + tRel
    V3D tRel = V3D(0, 0, 0);
    // relative angular velocity
    V3D angVelRel = V3D(0, 0, 0);
    // relative linear velocity
    // note. this is time derivative of tRel
    V3D velRel = V3D(0, 0, 0);
};

/**
 * State of mocap skeleton,
 */
class MocapSkeletonState {
private:
    Quaternion rootQ = Quaternion::Identity();
    P3D rootPos = P3D(0, 0, 0);
    V3D rootVel = V3D(0, 0, 0);
    V3D rootAngVel = V3D(0, 0, 0);

    std::vector<JointState> jointStates;

public:
    MocapSkeletonState(MocapSkeleton *skeleton, bool useDefaultAngles = false);

    MocapSkeletonState(const MocapSkeletonState &other);

    void setJointCount(int jCount);

    void setRootPosition(P3D p);

    void setRootOrientation(Quaternion q);

    void setRootVelocity(V3D v);

    void setRootAngularVelocity(V3D v);

    void setJointRelativeOrientation(const Quaternion &q, int jIdx);

    void setJointRelativeAngVelocity(V3D w, int jIdx);

    void setJointTranslation(const V3D &t, int jIdx);

    void setJointRelativeVelocity(V3D v, int jIdx);

    int getJointCount() const;

    P3D getRootPosition() const;

    Quaternion getRootOrientation() const;

    V3D getRootVelocity() const;

    V3D getRootAngularVelocity() const;

    Quaternion getJointRelativeOrientation(int jIdx) const;

    V3D getJointRelativeAngVelocity(int jIdx) const;

    V3D getJointTranslation(int jIdx) const;

    V3D getJointRelativeVelocity(int jIdx) const;
};

}  // namespace crl::mocap