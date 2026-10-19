
#include "MapManager/KeyframesLandmarks/Keyframe.h"

#include <algorithm>

// ============================================================
// CONSTRUCTOR
// ============================================================

Keyframe::Keyframe(
    KeyframeId id,
    Timestamp timestamp,
    const CameraPose& pose)
    : id_(id),
      timestamp_(timestamp),
      pose_(pose)
{
}

// ============================================================
// GETTERS
// ============================================================

KeyframeId Keyframe::getId() const
{
    return id_;
}

Timestamp Keyframe::getTimestamp() const
{
    return timestamp_;
}

const CameraPose& Keyframe::getPose() const
{
    return pose_;
}

// ============================================================
// POSE MANAGEMENT
// ============================================================

void Keyframe::setPose(const CameraPose& pose)
{
    pose_ = pose;
}

// ============================================================
// OBSERVATION MANAGEMENT
// ============================================================

void Keyframe::addObservation(
    const LandmarkObservation& observation)
{
    observations_.push_back(observation);
}

const std::vector<LandmarkObservation>&
Keyframe::getObservations() const
{
    return observations_;
}

bool Keyframe::removeObservationByLandmarkId(LandmarkId id)
{
    const auto oldSize = observations_.size();

    observations_.erase(
        std::remove_if(
            observations_.begin(),
            observations_.end(),
            [id](const LandmarkObservation& observation)
            {
                return observation.getLandmarkId() == id;
            }),
        observations_.end());

    return observations_.size() != oldSize;
}
