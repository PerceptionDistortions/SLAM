#pragma once

#include "Sensors/Calibration/DataStructures/CameraCalibration.h"
#include "Backend/Solver/OptimizationSolver.h"
#include "MapManager/KeyframesLandmarks/Keyframe.h"

#include <Eigen/Core>
#include <vector>

class PoseOnlyOptimization {
public:
    bool optimize(CameraPose& pose, const std::vector<Eigen::Vector3d>& worldPoints,
        const std::vector<Eigen::Vector2d>& measurements,
        const CameraCalibration& calibration, OptimizationSolver& solver);
};