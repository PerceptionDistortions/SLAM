#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};