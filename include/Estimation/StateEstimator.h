#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct StateEstimate
{
    double timestamp{0.0};

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};