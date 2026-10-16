#pragma once
#include<Eigen/Dense>

struct CameraPose
{
    // Camera-to-world rotation.
    Eigen::Matrix3d worldFromCameraRotation =Eigen::Matrix3d::Identity();

    // Camera center in world coordinates.
    Eigen::Vector3d positionWorld =Eigen::Vector3d::Zero();

    bool valid = false;
};