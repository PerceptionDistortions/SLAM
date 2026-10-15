
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

#include "Frontend/VisualFrontend/PoseOnlyRefinement/PoseCorrespondence.h"
#include "Frontend/VisualFrontend/PoseOnlyRefinement/PoseRefinementResult.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"

class IPoseRefiner
{
public:
    virtual ~IPoseRefiner() = default;

    // World-to-camera pose convention:
    // pointCamera = qCameraWorld * pointWorld + tCameraWorld.
    //
    // Updates the pose only when refinement succeeds.
    // Landmarks and calibration remain fixed.
    [[nodiscard]] virtual PoseRefinementResult refinePose(
        Eigen::Quaterniond& qCameraWorld,
        Eigen::Vector3d& tCameraWorld,
        const std::vector<PoseCorrespondence>& correspondences,
        const CameraCalibration& calibration) const = 0;
};
