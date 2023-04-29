//
// Created by Dongho Kang on 08.02.22.
//

#ifndef CRL_MOCAP_MOCAPMARKERS_H
#define CRL_MOCAP_MOCAPMARKERS_H

#include "mocap/MocapMarkerState.h"
#include "mocap/MocapModel.h"
#include "mocap/c3d/c3dfile.h"

namespace crl::mocap {

class MocapMarkers;

class MocapMarkersState {
private:
    std::map<std::string, MocapMarkerState> markerStates;

public:
    explicit MocapMarkersState(MocapMarkers *markers);

    MocapMarkersState(const MocapMarkersState &other);

    void setMarkerState(const MocapMarkerState &state, const std::string &markerName);

    MocapMarkerState getMarkerState(const std::string &markerName) const;
};

class MocapMarkers : public MocapModel<MocapMarkersState> {
public:
    explicit MocapMarkers(const char *filePath) {
        crl::mocap::C3DFile c3dfile;
        bool result = c3dfile.load(filePath);
        if (!result) {
            throw std::runtime_error("Cannot load c3d file: " + std::string(filePath));
        }
        std::vector<std::string> labels = c3dfile.point_label;
        for (const auto &label : labels) {
            markers.push_back(new MocapMarker());
            markers.back()->name = label;
        }
    }

    ~MocapMarkers() override = default;

    void populateState(MocapMarkersState *state, bool useDefaultAngles = false) override {
        for (const auto m : markers) {
            state->setMarkerState(m->state, std::string(m->name));
        }
    }

    void setState(const MocapMarkersState *state) override {
        for (const auto m : markers) {
            m->state = state->getMarkerState(m->name);
        }
    }
};

}  // namespace crl::mocap

#endif  //CRL_MOCAP_MOCAPMARKERS_H
