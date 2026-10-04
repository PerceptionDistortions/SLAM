#pragma once

#include <vector>

#include "MapTypes.h"
#include "Pose.h"
#include "LandmarkObservation.h"

class KeyFrame
{
public:
    KeyFrame(
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

private:
    KeyframeId id_;
    Timestamp timestamp_;

    Pose pose_;

    std::vector<LandmarkObservation>
        observations_;
};