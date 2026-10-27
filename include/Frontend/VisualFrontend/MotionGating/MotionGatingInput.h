#pragma once

#include <opencv2/core.hpp>
#include"Frontend/VisualFrontend/MotionGating/MotionEstimationType.h"


struct MotionGatingInput
{
    // Number of candidate correspondences supplied to the selected estimator.
    int candidateMatchCount = 0;

    // Number of RANSAC inliers from the selected estimator.
    int ransacInlierCount = 0;

    // Number of correspondences surviving recoverPose().
    int recoverPoseInlierCount = 0;

    // Relative rotation and translation from recoverPose().
    cv::Mat rotation;
    cv::Mat translation;

    MotionEstimationType method =
    MotionEstimationType::EssentialMatrix;

    double meanReprojectionError = 0.0;
    int positiveDepthCount = 0;
};
