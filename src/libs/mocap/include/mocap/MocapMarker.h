//
// Created by Dongho Kang on 28.12.21.
//

#ifndef CRL_MOCAP_MOCAPLABEL_H
#define CRL_MOCAP_MOCAPLABEL_H

#include <string>

#include "crl-basic/gui/renderer.h"
#include "crl-basic/utils/geoms.h"
#include "crl-basic/utils/mathDefs.h"
#include "mocap/MocapMarkerState.h"

namespace crl::mocap {

#define UPDATE_RAY_INTERSECTION(P1, P2)                                        \
    if (ray.getDistanceToSegment(P1, P2, &tmpIntersectionPoint) < cylRadius) { \
        double t = ray.getRayParameterFor(tmpIntersectionPoint);               \
        if (t < tMin) {                                                        \
            intersectionPoint = ray.origin + ray.dir * t;                      \
            tMin = t;                                                          \
        }                                                                      \
    }

struct MocapEndSite {
    std::string name;
    // from joint
    P3D endSiteOffset = P3D(0, 0, 0);
};

/**
 * MocapMarker has information of a relative transformation w.r.t a parent
 */
class MocapMarker {
public:
    std::string name;

    MocapMarker *parent = nullptr;
    // offset from parent in parent's frame
    P3D offset = P3D(0, 0, 0);
    std::vector<MocapMarker *> children;
    std::vector<MocapEndSite> endSites;

    // state
    MocapMarkerState state;

    // props
    bool selected = false;

public:
    Quaternion computeRelativeOrientation() {
        // if qp is the quaternion that gives the orientation of the parent, and qc
        // gives the orientation of the child, then  qp^-1 * qc gives the relative
        // orientation between the child and the parent (child to parent)
        if (!parent)
            return state.orientation.normalized();
        return (parent->state.orientation.inverse() * state.orientation).normalized();
    }

    V3D computeTranslation() const {
        if (!parent)
            return V3D(state.pos);
        V3D tmp = parent->state.orientation.inverse() * V3D(parent->state.pos, state.pos);
        return tmp - V3D(offset);
    }

    bool getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint) {
        P3D tmpIntersectionPoint;
        double tMin = DBL_MAX;
        double t = tMin;

        double cylRadius = 0.01;

        // Get joint world position
        P3D startPos = state.pos;

        for (auto *child : children) {
            UPDATE_RAY_INTERSECTION(startPos, child->state.pos);
        }

        for (const auto &ee : endSites) {
            UPDATE_RAY_INTERSECTION(startPos, state.getWorldCoordinates(ee.endSiteOffset));
        }

        return tMin < DBL_MAX / 2.0;
    }
};

}  // namespace crl::mocap

#endif  //CRL_MOCAP_MOCAPLABEL_H
