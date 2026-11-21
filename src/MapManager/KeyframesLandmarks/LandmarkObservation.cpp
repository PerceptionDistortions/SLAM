#include "MapManager/KeyframesLandmarks/LandmarkObservation.h"

LandmarkObservation::LandmarkObservation(
    KeyframeId keyframeId,
    FeatureId featureId,
    LandmarkId landmarkId,
    const Eigen::Vector2d& measurement)
    : keyframeId_(keyframeId),
      featureId_(featureId),
      landmarkId_(landmarkId),
      measurement_(measurement)
{
}

KeyframeId LandmarkObservation::getKeyframeId() const
{
    return keyframeId_;
}

FeatureId LandmarkObservation::getFeatureId() const
{
    return featureId_;
}

LandmarkId LandmarkObservation::getLandmarkId() const
{
    return landmarkId_;
}

const Eigen::Vector2d& LandmarkObservation::getMeasurement() const
{
    return measurement_;
}
