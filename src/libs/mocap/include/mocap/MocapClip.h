#pragma once

#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeleton.h"
#include "mocap/MocapSkeletonState.h"
#include "omp.h"
#include "mocap/MotionInterpolator.h"

namespace crl::mocap {

/**
 * this corresponds to one mocap (bvh) file.
 */
template <typename MocapModelType>
class MocapClip {
public:
    using StateType = typename MocapModelType::MocapModelStateType;

protected:
    std::string name_;
    double frameTimeStep_ = 1.0 / 60;
    uint frameCount_ = 0;

    // at the moment, we just assume there's only one skeleton in mocap.
    MocapModelType *model_ = nullptr;

    // 1 frame = 1 motion
    std::vector<StateType> motions_;

    /*!< interpolator */
    MotionInterpolator *interpolator;

public:
    virtual ~MocapClip() {
        delete model_;
        delete interpolator;
    }

    bool operator<(const MocapClip &other) const {
        return this->name_ < other.name_;
    }

    MocapModelType *getModel() {
        return model_;
    }

    const StateType &getState(uint frameIdx) const {
        return motions_[frameIdx];
    }

    uint getFrameCount() const {
        return frameCount_;
    }

    std::string getName() const {
        return name_;
    }

    double getFrameTimeStep() const {
        return frameTimeStep_;
    }

    void draw(const gui::Shader &shader, uint frameIdx, float alpha = 1.0) {
        if (frameIdx >= frameCount_) {
            Logger::consolePrint(V3D(1, 0, 0), "Wrong frame index %d > total frame count in clip = %d\n", frameIdx, frameCount_);
            frameIdx %= frameCount_;
        }

        // set skeleton
        if (model_) {
            model_->setState(&getState(frameIdx));
            model_->draw(shader, alpha);
        }
    }
};

class BVHClip : public MocapClip<MocapSkeleton> {
public:
    // explicit BVHClip(const std::string &filePath) {
    explicit BVHClip(const std::filesystem::path &filePath) {
        loadFromFile(filePath);
    }

    ~BVHClip() override = default;

private:
    void loadFromFile(const std::filesystem::path &mocapPath) {
        if (mocapPath.extension() == ".bvh")
            loadFromBVH(mocapPath);
        else
            // throw std::runtime_error("Not a supported file format: " + std::string(mocapPath.extension()));
            throw std::runtime_error("Not a supported file format: " + mocapPath.extension().string());

    }

    void loadFromBVH(const std::filesystem::path &mocapPath) {
        BvhParser parser;
        Bvh data;
        // if (parser.parse(std::string(mocapPath), &data)) {
        if (parser.parse(mocapPath.string(), &data)) {
            // throw std::runtime_error("Cannot parse bvh file: " + std::string(mocapPath));
            throw std::runtime_error("Cannot parse bvh file: " + mocapPath.string());
        }

        // load skeleton
        // model_ = new MocapSkeleton(mocapPath.c_str());
        model_ = new MocapSkeleton(mocapPath.string().c_str());

        /*!< create interpolator */
        interpolator = new MotionInterpolator(model_, 1.0/30.0, 1.0/30.0);

        // load motion
        auto bvhJoints = data.getJoints();
        double dt = data.getFrameTime()/2.0; /*!< upsample to 60fps */

        // name_ = std::string(mocapPath.filename());
        name_ = mocapPath.filename().string();
        frameTimeStep_ = dt;

        // save frame into motion
        // need to start index = 1 to compute velocity by FD
        motions_.reserve(2*data.getFramesCount()-3);
        for (int i = 0; i < motions_.capacity(); i++) motions_.emplace_back(model_);

        /*!< original motion pass */
        // #pragma omp parallel for
        for (int i = 1; i < data.getFramesCount(); i++) {
            auto bvhRoot = data.getRootJoint();

            auto previousT = bvhRoot->getLtwTransformAtFrame(i - 1);
            auto currentT = bvhRoot->getLtwTransformAtFrame(i);

            // motions_.emplace_back(model_);

            // root state
            V3D rootVel = V3D(currentT.translation() - previousT.translation()) / dt;
            V3D rootAngVel = estimateAngularVelocity(Quaternion(previousT.rotation()).normalized(), Quaternion(currentT.rotation()).normalized(), dt);

            motions_[2*(i-1)].setRootPosition(P3D() + currentT.translation());
            motions_[2*(i-1)].setRootOrientation(Quaternion(currentT.rotation()).normalized());
            motions_[2*(i-1)].setRootVelocity(rootVel);
            motions_[2*(i-1)].setRootAngularVelocity(rootAngVel);

            // joint state
            for (uint j = 0; j < bvhJoints.size(); j++) {
                auto previousLtp = bvhJoints[j]->getLtpTransformAtFrame(i - 1);
                auto currentLtp = bvhJoints[j]->getLtpTransformAtFrame(i);

                Quaternion qRel = Quaternion(currentLtp.rotation()).normalized();
                V3D tRel = V3D(currentLtp.translation());
                // TODO: is angvelrel correct here?
                V3D angVelRel = estimateAngularVelocity(Quaternion(previousLtp.rotation()).normalized(), Quaternion(currentLtp.rotation()).normalized(), dt);
                V3D velRel = V3D(currentLtp.translation() - previousLtp.translation()) / dt;

                motions_[2*(i-1)].setJointRelativeOrientation(qRel, j);
                motions_[2*(i-1)].setJointTranslation(tRel, j);
                motions_[2*(i-1)].setJointRelativeAngVelocity(angVelRel, j);
                motions_[2*(i-1)].setJointRelativeVelocity(velRel, j);
            }

            // update frame count
            // #pragma omp critical
            frameCount_++;
        }
        
        /*!> motion interpolation pass */
        for (int i = 0; i < data.getFramesCount()-2; i++) {
            int idxInterp = 2*i+1; /*!< interpolation position */
            int idxLastLast, idxLast, idxNext, idxNextNext;
            if (i==0) { idxLastLast=0; idxLast=0; idxNext=2; idxNextNext=4; }
            else if (i== data.getFramesCount()-3) {idxLastLast=idxInterp-3; idxLast=idxInterp-1; idxNext=idxInterp+1; idxNextNext=idxInterp+1;}
            else { idxLastLast=idxInterp-3; idxLast=idxInterp-1; idxNext=idxInterp+1; idxNextNext=idxInterp+3; }

            interpolator->blend(motions_[idxLastLast], motions_[idxLast], motions_[idxNext], motions_[idxNextNext]);
            auto interpState = interpolator->evaluate(1.0/60.0);

            motions_[idxInterp].setRootPosition(interpState->getRootPosition());
            motions_[idxInterp].setRootOrientation(interpState->getRootOrientation());
            /*!< use FD to approximate velocity */
            // motions_[idxInterp].setRootVelocity(V3D(motions_[idxInterp-1].getRootPosition(), interpState->getRootPosition())/dt);
            // motions_[idxInterp].setRootAngularVelocity(estimateAngularVelocity(motions_[idxInterp-1].getRootOrientation(), interpState->getRootOrientation(), dt));

            for (int j = 0; j < bvhJoints.size(); j++) {
                motions_[idxInterp].setJointRelativeOrientation(interpState->getJointRelativeOrientation(j), j);
                // motions_[idxInterp].setJointRelativeAngVelocity(estimateAngularVelocity(motions_[idxInterp-1].getJointRelativeOrientation(j), interpState->getJointRelativeOrientation(j), dt), j);
            }

            frameCount_++;
        }

        /*!> velocity FD pass */
        for (int i = 1; i < data.getFramesCount(); i++) {
            motions_[i].setRootVelocity(V3D(motions_[i-1].getRootPosition(), motions_[i].getRootPosition())/dt);
            motions_[i].setRootAngularVelocity(estimateAngularVelocity(motions_[i-1].getRootOrientation(), motions_[i].getRootOrientation(), dt));

            for (int j = 0; j < bvhJoints.size(); j++) {
                motions_[i].setJointRelativeAngVelocity(estimateAngularVelocity(motions_[i-1].getJointRelativeOrientation(j), motions_[i].getJointRelativeOrientation(j), dt), j);
            }

        }    

    }
};

class C3DClip : public MocapClip<MocapMarkers> {
public:
    // explicit C3DClip(const std::string &filePath) {
    explicit C3DClip(const std::filesystem::path &filePath) {
        loadFromFile(filePath);
    }

    ~C3DClip() override = default;

private:
    void loadFromFile(const std::filesystem::path &mocapPath) {
        if (mocapPath.extension() == ".c3d")
            loadFromC3D(mocapPath);
        else
            // throw std::runtime_error("Not a supported file format: " + std::string(mocapPath.extension()));
            throw std::runtime_error("Not a supported file format: " + mocapPath.extension().string());
    }

    void loadFromC3D(const std::filesystem::path &mocapPath) {
        crl::mocap::C3DFile c3dfile;
        // bool result = c3dfile.load(mocapPath.c_str());
        bool result = c3dfile.load(mocapPath.string().c_str());
        if (!result) {
            // throw std::runtime_error("Cannot load c3d file: " + std::string(mocapPath));
            throw std::runtime_error("Cannot load c3d file: " + mocapPath.string());
        }

        // props
        // name_ = std::string(mocapPath.filename());
        name_ = mocapPath.filename().string();
        frameTimeStep_ = 1.0 / c3dfile.header.video_sampling_rate;

        // unit
        std::string pointUnits = c3dfile.getParamString("POINT:UNITS");
        double unit = 1.0;
        if (pointUnits == "mm")
            unit = 1e-3;

        // markers
        std::vector<std::string> labels = c3dfile.point_label;
        // model_ = new MocapMarkers(mocapPath.c_str());
        model_ = new MocapMarkers(mocapPath.string().c_str());
        // note. first_frame is usually 1 and last_frame is N frame.
        for (int i = c3dfile.header.first_frame; i <= c3dfile.header.last_frame; i++) {
            motions_.emplace_back(model_);
        }

        for (const auto &label : labels) {
            // marker motion
            const auto &markerTraj = c3dfile.getMarkerTrajectories(label.c_str());

            // we start from the second frame for computing derivative
            for (int i = 0; i < markerTraj.x.size(); i++) {
                // store state
                MocapMarkerState state;

                // pos
                // change to y-up axis
                P3D p(markerTraj.y[i], markerTraj.z[i], markerTraj.x[i]);
                if (markerTraj.x[i] == 0 && markerTraj.y[i] == 0 && markerTraj.z[i] == 0) {
                    // occlusion! (just use previous frame)
                    state.pos = motions_[i - 1].getMarkerState(label).pos;
                } else {
                    state.pos = p * unit;
                }

                // vel
                if (i > 0)
                    state.velocity = V3D(motions_[i - 1].getMarkerState(label).pos, state.pos) / frameTimeStep_;
                motions_[i].setMarkerState(state, label);
            }
        }

        // remove the very first motion (invalid velocity)
        motions_.erase(motions_.begin());

        // update frame count
        frameCount_ = motions_.size();
    }
};

}  // namespace crl::mocap
