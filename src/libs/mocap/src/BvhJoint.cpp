#include "mocap/bvh/BvhJoint.h"

namespace crl::mocap {

#define UPDATE_RAY_INTERSECTION(P1, P2)                                        \
    if (ray.getDistanceToSegment(P1, P2, &tmpIntersectionPoint) < cylRadius) { \
        double t = ray.getRayParameterFor(tmpIntersectionPoint);               \
        if (t < tMin) {                                                        \
            intersectionPoint = ray.origin + ray.dir * t;                      \
            tMin = t;                                                          \
        }                                                                      \
    }

void BvhJoint::addFrameMotionData(const std::vector<float> &data) {
    // Sanity check
    if (data.empty()) {
        std::cout << "Joint - WARNING: empty channel data vector" << std::endl;
        return;
    }

    // Add motion data
    channel_data_.push_back(data);

    // Add local to world transformation object
    if (parent_ == nullptr) {
        // Root joint

        // Initialize empty transform
        Transform_t ltw;

        // Create local to parent (initial pose?) transform from channel data
        Transform_t ltp = getTransformFromMotionData(data[0], data[1], data[2],
                                                     data[3], data[4], data[5]);

        // Now chain offset translation and ltp to obtain ltw
        ltw = Eigen::Translation3d(offset_) * ltp;

        // FIXME: pre-allocation with assignment instead of push_back
        ltpTransforms_.push_back(ltp);
        ltwTransforms_.push_back(ltw);
    }
    // Any other joint
    else {
        Transform_t ltw;

        // ltwTransform from the parent joint
        unsigned this_frame_idx =
            static_cast<unsigned>(channel_data_.size() - 1);
        Transform_t pt = parent_->getLtwTransformAtFrame(this_frame_idx);

        // Translation due to fixed offset
        Transform_t offsetTransl;
        offsetTransl = Eigen::Translation3d(offset_);

        // Build local to parent transform from channel data
        Transform_t ltp = getTransformFromMotionData(data[0], data[1], data[2]);

        // Save ltp transform
        ltpTransforms_.push_back(ltp);

        // Chain transforms
        ltw = pt * offsetTransl * ltp;

        // Add to vector
        // FIXME: pre-allocation with assignment instead of push_back
        ltwTransforms_.push_back(ltw);
    }
}

crl::V3D BvhJoint::getJointWorldPosAtFrame(unsigned frame) {
    // Corner case - root joint
    if (parent_ == nullptr) {
        // Get the position data at frame and create a translation vector
        crl::V3D translationVec(channel_data_[frame][0],
                                channel_data_[frame][1],
                                channel_data_[frame][2]);
        // Apply the translation vector to the root offset and return
        crl::V3D world = scale_ * translationVec + offset_;
        return world;
    }

    // Get parent ltw at frame
    Transform_t parentLtw = parent_->getLtwTransformAtFrame(frame);

    // Apply to offset from parent and return
    crl::V3D world = parentLtw * offset_;
    return world * scale_;
}

bool BvhJoint::getRayIntersectionPoint(const crl::Ray &ray,
                                       crl::P3D &intersectionPoint,
                                       unsigned frameNo) {
    crl::P3D tmpIntersectionPoint;
    double tMin = DBL_MAX;
    double t = tMin;

    double cylRadius = 0.01;

    // Get joint world position
    crl::P3D startPos = getP3D(getJointWorldPosAtFrame(frameNo));

    for (const JointPtr child : getChildren())
        UPDATE_RAY_INTERSECTION(
            startPos, getP3D(child->getJointWorldPosAtFrame(frameNo)));

    for (auto ee : getEndEffectors())
        UPDATE_RAY_INTERSECTION(startPos,
                                getP3D(getWorldPosAtFrame(frameNo, ee)));

    return tMin < DBL_MAX / 2.0;
}

}  // namespace crl::mocap