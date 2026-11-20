#pragma once

#include "Sensors/Calibration/DataStructures/CameraCalibration.h"
#include "Backend/Solver/OptimizationSolver.h"

#include <Eigen/Core>
#include <vector>

class EssentialMatrixOptimization {
public:
    bool optimize(Eigen::Matrix3d& relativeRotation,
        Eigen::Vector3d& translationDirection,
        std::vector<Eigen::Vector3d>& landmarks3D,
        const std::vector<Eigen::Vector2d>& points1,
        const std::vector<Eigen::Vector2d>& points2,
        const CameraCalibration& calibration,
        OptimizationSolver& solver);
};