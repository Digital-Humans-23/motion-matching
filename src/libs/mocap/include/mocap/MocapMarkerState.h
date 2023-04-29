//
// Created by Dongho Kang on 28.12.21.
//

#ifndef CRL_MOCAP_MOCAPMARKERSTATE_H
#define CRL_MOCAP_MOCAPMARKERSTATE_H

#include "utils/mathDefs.h"
#include "utils/mathUtils.h"

namespace crl::mocap {

class MocapMarkerState {
public:
    // joint position in world
    P3D pos = P3D(0, 0, 0);
    // joint orientation in world
    Quaternion orientation = Quaternion::Identity();

    // these values are updated by finite difference!
    // the velocity of the joint, in world coords
    V3D velocity = V3D(0, 0, 0);
    // and finally, the angular velocity in world coords
    V3D angularVelocity = V3D(0, 0, 0);

public:
    MocapMarkerState() = default;

    MocapMarkerState(const MocapMarkerState &other) {
        this->pos = other.pos;
        this->orientation = other.orientation;
        this->velocity = other.velocity;
        this->angularVelocity = other.angularVelocity;
    }

    MocapMarkerState &operator=(const MocapMarkerState &other) {
        this->pos = other.pos;
        this->orientation = other.orientation;
        this->velocity = other.velocity;
        this->angularVelocity = other.angularVelocity;
        return *this;
    }

    bool operator==(const MocapMarkerState &other) const {
        if (V3D(pos, other.pos).norm() > 1e-10)
            return false;
        //q and -q represent the same rotation...
        if (!sameRotation(orientation, other.orientation))
            return false;
        if ((velocity - other.velocity).norm() > 1e-10)
            return false;
        if ((angularVelocity - other.angularVelocity).norm() > 1e-10)
            return false;
        return true;
    }

    inline P3D getWorldCoordinates(const P3D &pLocal) const {
        // pWorld = pos + R * V3D(origin, pLocal)
        return pos + orientation * V3D(pLocal);
    }

    inline V3D getWorldCoordinates(const V3D &vLocal) const {
        // the rigid body's orientation is a unit quaternion. Using this, we can
        // obtain the global coordinates of a local vector
        return orientation * vLocal;
    }

    inline P3D getLocalCoordinates(const P3D &pWorld) const {
        return P3D() + orientation.inverse() * (V3D(pos, pWorld));
    }

    inline V3D getLocalCoordinates(const V3D &vWorld) const {
        // the rigid body's orientation is a unit quaternion. Using this, we can
        // obtain the global coordinates of a local vector
        return orientation.inverse() * vWorld;
    }

    // TODO: check! what about pLocal is time varying?
    inline V3D getVelocityForPoint_local(const P3D &pLocal) const {
        // we need to compute the vector r, from the origin of the body to the
        // point of interest
        V3D r = V3D(pLocal);
        // the velocity is given by omega x r + v. omega and v are already
        // expressed in world coordinates, so we need to express r in world
        // coordinates first.
        return angularVelocity.cross(getWorldCoordinates(r)) + velocity;
    }
};

}  // namespace crl::mocap

#endif  //CRL_MOCAP_MOCAPMARKERSTATE_H
