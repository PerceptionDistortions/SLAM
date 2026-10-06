#pragma once

#include <Eigen/Core>

#include "MapManager/MapTypes.h"

class Landmark
{
public:
    Landmark(
        LandmarkId id,
        const Eigen::Vector3d& position);

    LandmarkId getId() const;

    const Eigen::Vector3d& getPosition() const;
    void setPosition(const Eigen::Vector3d& position);

    bool isValid() const;
    void setValid(bool valid);

private:
    LandmarkId id_;
    Eigen::Vector3d position_;
    bool valid_{true};
};