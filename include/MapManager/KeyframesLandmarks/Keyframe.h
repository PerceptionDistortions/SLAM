
#pragma once

#include <vector>

#include "AliasesEnumsTypes/CommonAliases.h"
#include "AliasesEnumsTypes/CameraPose.h"
#include "MapManager/KeyframesLandmarks/LandmarkObservation.h"

class Keyframe
{
public:
    Keyframe(
        KeyframeId id,
        Timestamp timestamp,
        const CameraPose& pose);

    KeyframeId getId() const;
    Timestamp getTimestamp() const;

    const CameraPose& getPose() const;
    void setPose(const CameraPose& pose);

    void addObservation(const LandmarkObservation& observation);
    const std::vector<LandmarkObservation>& getObservations() const;

    bool removeObservationByLandmarkId(LandmarkId id);

private:
    KeyframeId id_;
    Timestamp timestamp_;
    CameraPose pose_;
    std::vector<LandmarkObservation> observations_;
};
