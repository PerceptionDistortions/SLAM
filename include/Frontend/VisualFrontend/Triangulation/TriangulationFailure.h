#pragma once

#include <Eigen/Core>

#include <opencv2/core/types.hpp>

enum class TriangulationFailure
{
    None,
    InvalidInput,
    InvalidCalibration,
    InvalidPose,
    InsufficientBaseline,
    InsufficientParallax,
    DegenerateGeometry,
    NegativeDepth,
    ExcessiveReprojectionError,
    ExcessiveRaySeparation
};

struct TriangulationObservation
{
    cv::Point2d pixel;

    // World-to-camera transformation: T_cw.
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();

    // Camera intrinsic matrix.
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
};

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
