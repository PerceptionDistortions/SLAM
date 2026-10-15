#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <vector>

struct PnPResult
{
    bool success = false;

    // World/map-to-camera transformation:
    // P_camera = R * P_world + t
    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat rotationVector;

    // Indices refer to the original input correspondences.
    std::vector<int> inlierIndices;

    // One entry per input correspondence: 1 = inlier, 0 = outlier.
    std::vector<std::uint8_t> inlierMask;

    int candidateCount = 0;
    int inlierCount = 0;
    int positiveDepthCount = 0;

    double meanReprojectionError = 0.0;
};