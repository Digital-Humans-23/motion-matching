#pragma once

#include <crl-basic/gui/renderer.h>

#include "mocap/MocapMarker.h"

namespace crl::mocap {

/**
 * This class is a counter part of RBRenderer dedicated for Bvh Joints. It has
 * static member functions that render joints, and relevant informations.
 */
class MocapRenderer {
public:
    /* Rendering params */

    // The default radius of the cylinders representing the bones
    static constexpr double linkCylinderRadius_ = 0.01;

    // Default colors
    static V3D defaultLinkDrawColor_;
    static V3D selectedLinkDrawColor_;
    static V3D jointDrawColor_;
    static V3D defaultEndSiteLinkColor_;
    static V3D selectedEndSiteLinkColor_;
    static V3D endSiteContactColor_;

public:
    static void drawMarker(const MocapMarker *joint, const crl::gui::Shader &shader);

    static void drawCoordFrame(const MocapMarker *joint, const gui::Shader &shader);

    static void drawJointVelocity(const MocapMarker *joint, const gui::Shader &shader);
};

}  // namespace crl::mocap