//
// Created by Dongho Kang on 08.02.22.
//
#pragma once

#include "mocap/MocapMarker.h"
#include "mocap/MocapRenderer.h"

namespace crl::mocap {

/**
 * Mocap model is a collection of mocap markers
 */
template <typename T>
class MocapModel {
public:
    using MocapModelStateType = T;

    // drawing flags
    bool showCoordFrame = false;
    bool showVelocity = false;

    // list of all joint belongs to this skeloton
    std::vector<MocapMarker *> markers;

public:
    virtual ~MocapModel() {
        for (uint i = 0; i < markers.size(); i++)
            delete markers[i];
        markers.clear();
    }

    virtual void populateState(MocapModelStateType *state, bool useDefaultAngles = false) = 0;

    virtual void setState(const MocapModelStateType *state) = 0;

    virtual MocapMarker *getMarkerByName(const char *jName) {
        for (uint i = 0; i < markers.size(); i++) {
            if (strcmp(markers[i]->name.c_str(), jName) == 0)
                return markers[i];
        }
        std::cout << "WARNING: MocapModel:getMarkerByName -> marker could not be found..." << std::endl;
        return nullptr;
    }

    virtual int getMarkerCount() const {
        return (int)markers.size();
    }

    virtual MocapMarker *getMarker(int i) const {
        if (i < 0 || i > (int)markers.size() - 1)
            return nullptr;
        return markers[i];
    }

    virtual void draw(const gui::Shader &shader, float alpha = 1.0) const {
        for (uint i = 0; i < markers.size(); i++)
            MocapRenderer::drawMarker(markers[i], shader);

        if (showCoordFrame) {
            for (uint i = 0; i < markers.size(); i++)
                MocapRenderer::drawCoordFrame(markers[i], shader);
        }

        if (showVelocity) {
            for (uint i = 0; i < markers.size(); i++)
                MocapRenderer::drawJointVelocity(markers[i], shader);
        }
    }

    virtual MocapMarker *getFirstJointHitByRay(const Ray &ray, P3D &intersectionPoint) {
        MocapMarker *selectedJoint = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (const auto &joint : markers) {
            if (joint->getRayIntersectionPoint(ray, tmpIntersectionPoint)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedJoint = joint;
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }

        return selectedJoint;
    }
};

}  // namespace crl::mocap
