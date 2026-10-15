#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

struct CameraExtrinsics
{
    // Rotation from the rig frame to the camera frame.
    // Convention: p_camera = q_camera_rig * p_rig + t_camera_rig
    Eigen::Quaterniond q_camera_rig =
        Eigen::Quaterniond::Identity();

    // Translation of the rig origin expressed in the camera frame.
    Eigen::Vector3d t_camera_rig =
        Eigen::Vector3d::Zero();

    [[nodiscard]] Eigen::Vector3d transformRigToCamera(
        const Eigen::Vector3d& pointRig) const
    {
        return q_camera_rig * pointRig + t_camera_rig;
    }

    [[nodiscard]] Eigen::Vector3d transformCameraToRig(
        const Eigen::Vector3d& pointCamera) const
    {
        return q_camera_rig.conjugate() *
               (pointCamera - t_camera_rig);
    }

    [[nodiscard]] bool isValid() const
    {
        return q_camera_rig.coeffs().allFinite() &&
               t_camera_rig.allFinite() &&
               std::isfinite(q_camera_rig.norm()) &&
               q_camera_rig.norm() > 1e-12;
    }

    void normalizeRotation()
    {
        const double norm = q_camera_rig.norm();

        if (std::isfinite(norm) && norm > 1e-12)
        {
            q_camera_rig.normalize();
        }
    }
};