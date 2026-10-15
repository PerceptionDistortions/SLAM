#include<Eigen/Dense>

#pragma once

#include <Eigen/Dense>

#include "Sensors/Calibration/DataStructures/CameraCalibration.h"
#include "Sensors/Calibration/CameraExtrinsics.h"

struct PoseObservation
{
    Eigen::Vector3d landmarkWorld =Eigen::Vector3d::Zero();

    Eigen::Vector2d imagePoint =Eigen::Vector2d::Zero();

    CameraCalibration intrinsics;
    CameraExtrinsics extrinsics;

    bool isValid() const
    {
        return landmarkWorld.allFinite() &&
               imagePoint.allFinite() &&
               intrinsics.fx > 0.0 &&
               intrinsics.fy > 0.0 &&
               std::isfinite(intrinsics.cx) &&
               std::isfinite(intrinsics.cy) &&
               extrinsics.q_camera_rig.coeffs().allFinite() &&
               extrinsics.t_camera_rig.allFinite() &&
               extrinsics.q_camera_rig.norm() > 1e-12;
    }
};