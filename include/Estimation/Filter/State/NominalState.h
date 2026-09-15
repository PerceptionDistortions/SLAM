#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct NominalState
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

    double timestamp{0.0};

    void normalizeOrientation();
};