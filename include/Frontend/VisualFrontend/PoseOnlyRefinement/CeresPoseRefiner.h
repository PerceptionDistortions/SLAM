
#pragma once

#include "Frontend/VisualFrontend/PoseOnlyRefinement/IPoseRefiner.h"

class CeresPoseRefiner final : public IPoseRefiner
{
public:
    [[nodiscard]] PoseRefinementResult refinePose(
        Eigen::Quaterniond& qCameraWorld,
        Eigen::Vector3d& tCameraWorld,
        const std::vector<PoseCorrespondence>& correspondences,
        const CameraCalibration& calibration) const override;
};
