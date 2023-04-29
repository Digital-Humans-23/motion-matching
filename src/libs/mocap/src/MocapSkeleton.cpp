#include "mocap/MocapSkeleton.h"

#include "mocap/MocapRenderer.h"
#include "mocap/MocapSkeletonState.h"

namespace crl::mocap {

MocapSkeleton::MocapSkeleton(const char *filePath) {
    // load skeleton from importer
    BvhParser parser;
    Bvh data;
    if (!parser.parse(std::string(filePath), &data)) {
        std::runtime_error("Cannot parse bvh file: " + std::string(filePath));
    }
    auto bvhJoints = data.getJoints();

    // create joints first
    for (uint i = 0; i < bvhJoints.size(); i++) {
        markers.push_back(new MocapMarker());
        markers.back()->name = bvhJoints[i]->getName();
        markers.back()->offset = P3D() + bvhJoints[i]->getOffset();

        // end effectors
        for (uint j = 0; j < bvhJoints[i]->getEndEffectors().size(); j++) {
            markers.back()->endSites.push_back(MocapEndSite());
            markers.back()->endSites.back().name = bvhJoints[i]->getName();
            markers.back()->endSites.back().endSiteOffset = P3D() + bvhJoints[i]->getEndEffectors()[j];
        }
    }

    // then add parent and children
    for (uint i = 0; i < bvhJoints.size(); i++) {
        auto bvhJoint = bvhJoints[i];
        auto *joint = getMarkerByName(bvhJoint->getName().c_str());

        // parent
        if (bvhJoint->getParent())
            joint->parent = getMarkerByName(bvhJoint->getParent()->getName().c_str());

        // children
        for (uint j = 0; j < bvhJoint->getChildren().size(); j++) {
            joint->children.push_back(getMarkerByName(bvhJoint->getChildren()[j]->getName().c_str()));
        }
    }

    // set root
    if (data.getRootJoint())
        root = getMarkerByName(data.getRootJoint()->getName().c_str());

    // finally update each joint state by forward kinematics
    MocapSkeletonState s(this, true);
    setState(&s);
}

void MocapSkeleton::populateState(MocapSkeletonState *state, bool useDefaultAngles) {
    state->setRootPosition(root->state.pos);
    state->setRootOrientation(root->state.orientation);
    state->setRootVelocity(root->state.velocity);
    state->setRootAngularVelocity(root->state.angularVelocity);

    state->setJointCount(markers.size());
    for (uint i = 0; i < markers.size(); i++) {
        if (!useDefaultAngles) {
            // here order matters!
            state->setJointRelativeOrientation(markers[i]->computeRelativeOrientation(), i);
            state->setJointTranslation(markers[i]->computeTranslation(), i);
            state->setJointRelativeAngVelocity(getRelativeLocalCoordsAngularVelocityForJoint(markers[i]), i);
            state->setJointRelativeVelocity(getRelativeLocalCoordsVelocityForJoint(markers[i]), i);
        } else {
            state->setJointRelativeOrientation(Quaternion::Identity(), i);
            state->setJointTranslation(V3D(), i);
            state->setJointRelativeAngVelocity(V3D(), i);
            state->setJointRelativeVelocity(V3D(), i);
        }
    }
}

void MocapSkeleton::setState(const MocapSkeletonState *state) {
    root->state.pos = state->getRootPosition();
    root->state.orientation = state->getRootOrientation().normalized();
    root->state.velocity = state->getRootVelocity();
    root->state.angularVelocity = state->getRootAngularVelocity();

    for (uint i = 0; i < markers.size(); i++) {
        // order matters!
        setRelativeOrientationForJoint(markers[i], state->getJointRelativeOrientation(i).normalized());
        setTranslationForJoint(markers[i], state->getJointTranslation(i));
        setRelativeLocalCoordsAngularVelocityForJoint(markers[i], state->getJointRelativeAngVelocity(i));
        setRelativeLocalCoordsVelocityForJoint(markers[i], state->getJointRelativeVelocity(i));
    }
}

void MocapSkeleton::setRootState(const P3D &position, const Quaternion &orientation, const V3D &vel, const V3D &angVel) {
    MocapSkeletonState state(this);
    populateState(&state);
    state.setRootPosition(position);
    state.setRootOrientation(orientation);
    state.setRootVelocity(vel);
    state.setRootAngularVelocity(angVel);
    setState(&state);
}

}  // namespace crl::mocap