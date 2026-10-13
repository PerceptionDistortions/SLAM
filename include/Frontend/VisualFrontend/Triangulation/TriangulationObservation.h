#pragma once

#include <Eigen/Core>

#include <opencv2/core/types.hpp>

struct TriangulationObservation
{
    cv::Point2d pixel;

    // World-to-camera transformation: T_cw.
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();

    // Camera intrinsic matrix.
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
};