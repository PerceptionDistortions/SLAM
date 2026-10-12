#include "MapManager/KeyframesLandmarks/Landmark.h"

Landmark::Landmark(
    LandmarkId id,
    const Eigen::Vector3d& position)
    : id_(id),
      position_(position)
{
}

LandmarkId Landmark::getId() const
{
    return id_;
}

const Eigen::Vector3d& Landmark::getPosition() const
{
    return position_;
}

void Landmark::setPosition(const Eigen::Vector3d& position)
{
    position_ = position;
}

bool Landmark::isValid() const
{
    return valid_;
}

void Landmark::setValid(bool valid)
{
    valid_ = valid;
}