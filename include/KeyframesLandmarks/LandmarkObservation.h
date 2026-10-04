#pragma once

#include <Eigen/Core>

#include "MapTypes.h"

//A LANDMARK OBSERVED IN A KEYFRAME
//WHAT FEATURE ID IT CORRESONDS TO
//STORED BY A KEYFRAME
class LandmarkObservation
{
public:
    LandmarkObservation(
        KeyframeId keyframeId,
        FeatureId featureId,
        LandmarkId landmarkId,
        const Eigen::Vector2d& measurement);

    KeyframeId getKeyframeId() const;
    FeatureId getFeatureId() const;
    LandmarkId getLandmarkId() const;

    const Eigen::Vector2d& getMeasurement() const;

private:
    KeyframeId keyframeId_;
    FeatureId featureId_;
    LandmarkId landmarkId_;

    // Generic sensor measurement.
    // For a monocular camera this is typically (u, v).
    Eigen::Vector2d measurement_;
};