#pragma once

#include <Eigen/Core>

struct ErrorState
{
    Eigen::Vector3d positionError{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocityError{Eigen::Vector3d::Zero()};
    Eigen::Vector3d orientationError{Eigen::Vector3d::Zero()};

    void reset();
};