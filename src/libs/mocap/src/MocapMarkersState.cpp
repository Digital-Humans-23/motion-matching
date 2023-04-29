//
// Created by Dongho Kang on 08.02.22.
//

#include "mocap/MocapMarkers.h"

namespace crl::mocap {

MocapMarkersState::MocapMarkersState(MocapMarkers *markers) {
    markers->populateState(this);
}

MocapMarkersState::MocapMarkersState(const MocapMarkersState &other) {
    this->markerStates = other.markerStates;
}

void MocapMarkersState::setMarkerState(const MocapMarkerState &state, const std::string &markerName) {
    markerStates[markerName] = state;
}

MocapMarkerState MocapMarkersState::getMarkerState(const std::string &markerName) const {
    const auto it = markerStates.find(markerName);
    if (it == markerStates.end()) {
        return MocapMarkerState();  // just return identity
    }
    return it->second;
}

}  // namespace crl::mocap