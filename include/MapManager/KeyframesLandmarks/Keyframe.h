#pragma once

#include <vector>

#include "AliasesEnumsTypes/CommonAliases.h"
#include "MapManager/Pose.h"
#include "MapManager/KeyframesLandmarks/LandmarkObservation.h"

class Keyframe
{
public:
    Keyframe(
        KeyframeId id,
        Timestamp timestamp,
        const Pose& pose);

    KeyframeId getId() const;
    Timestamp getTimestamp() const;

    const Pose& getPose() const;
    void setPose(const Pose& pose);

    void addObservation(
        const LandmarkObservation& observation);

    const std::vector<LandmarkObservation>&
    getObservations() const;

    bool removeObservationByLandmarkId(LandmarkId id);

private:
    KeyframeId id_;
    Timestamp timestamp_;

    Pose pose_;

    std::vector<LandmarkObservation>
        observations_;
};