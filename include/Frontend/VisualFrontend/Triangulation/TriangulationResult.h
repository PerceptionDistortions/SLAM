#pragma once

#include <Eigen/Core>

#include <opencv2/core/types.hpp>

#include"Frontend/VisualFrontend/Triangulation/TriangulationFailure.h"

struct TriangulationResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d pointWorld = Eigen::Vector3d::Zero();

    double parallaxDegrees = 0.0;
    double raySeparation = 0.0;

    double reprojectionErrorFirst = 0.0;
    double reprojectionErrorSecond = 0.0;

    double depthFirst = 0.0;
    double depthSecond = 0.0;

    TriangulationFailure failure = TriangulationFailure::InvalidInput;

    [[nodiscard]] bool valid() const noexcept
    {
        return failure == TriangulationFailure::None;
    }
};