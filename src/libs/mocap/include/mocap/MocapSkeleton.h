#pragma once

#include <crl-basic/gui/renderer.h>
#include <crl-basic/utils/mathDefs.h>
#include <crl-basic/utils/utils.h>

#include <memory>
#include <vector>

#include "mocap/MocapMarker.h"
#include "mocap/MocapModel.h"
#include "mocap/bvh/BvhParser.h"

namespace crl::mocap {

class MocapSkeletonState;

/**
 * Hierarchy tree of mocap joints.
 */
class MocapSkeleton : public MocapModel<MocapSkeletonState> {
    friend class MocapSkeletonState;

public:
    MocapMarker *root = nullptr;

    // drawing flags
    bool showCoordFrame = false;
    bool showVelocity = false;

    // frame
    // up axis and the axis where the character is heading toward.
    V3D forwardAxis = V3D(1, 0, 0);
    V3D upAxis = V3D(0, 1, 0);

public:
    explicit MocapSkeleton(const char *filePath);

    ~MocapSkeleton() override = default;

    void populateState(MocapSkeletonState *state, bool useDefaultAngles = false) override;

    void setState(const MocapSkeletonState *state) override;

    void setRootState(const P3D &position = P3D(0, 0, 0), const Quaternion &orientation = Quaternion::Identity(), const V3D &vel = V3D(0, 0, 0),
                      const V3D &angVel = V3D(0, 0, 0));

    void setRelativeOrientationForJoint(MocapMarker *joint, const Quaternion &qRel) {
        if (!joint->parent)
            return;  // skip root
        joint->state.orientation = joint->parent->state.orientation * qRel;
    }

    void setTranslationForJoint(MocapMarker *joint, const V3D &t) {
        if (!joint->parent)
            return;  // skip root
        V3D tmp = V3D(joint->offset) + t;
        joint->state.pos = joint->parent->state.pos + joint->parent->state.orientation * tmp;
    }

    V3D getRelativeLocalCoordsAngularVelocityForJoint(MocapMarker *joint) {
        // we will store wRel in the parent's coordinates, to get an orientation
        // invariant expression for it
        if (!joint->parent)
            return joint->state.angularVelocity;
        return joint->parent->state.getLocalCoordinates(V3D(joint->state.angularVelocity - joint->parent->state.angularVelocity));
    }

    V3D getRelativeLocalCoordsVelocityForJoint(MocapMarker *joint) {
        // we will store wRel in the parent's coordinates, to get an orientation
        // invariant expression for it
        if (!joint->parent)
            return joint->state.velocity;
        // this is a bit tricky...
        // relVel = Rparent.inverse * (vWorld - vParentWorld - wParentWorld x (rWorld - rParentWorld))
        V3D tmp =
            joint->state.velocity - joint->parent->state.velocity - joint->parent->state.angularVelocity.cross(V3D(joint->parent->state.pos, joint->state.pos));
        return joint->parent->state.getLocalCoordinates(tmp);
    }

    void setRelativeLocalCoordsAngularVelocityForJoint(MocapMarker *joint, const V3D &relAngVel) {
        if (!joint->parent)
            return;  // skip root
        // assume relAngVel is stored in the parent's coordinate frame, to get
        // an orientation invariant expression for it
        joint->state.angularVelocity = joint->parent->state.angularVelocity + joint->parent->state.getWorldCoordinates(relAngVel);
    }

    void setRelativeLocalCoordsVelocityForJoint(MocapMarker *joint, const V3D &relVel) {
        if (!joint->parent)
            return;  // skip root
        // this is a bit tricky again
        // vWorld = vParentWorld + wWorld x (rWorld - rParentWorld) + Rparent * relVel
        joint->state.velocity = joint->parent->state.velocity + joint->parent->state.angularVelocity.cross(V3D(joint->parent->state.pos, joint->state.pos)) +
                                joint->parent->state.getWorldCoordinates(relVel);
    }
};

}  // namespace crl::mocap