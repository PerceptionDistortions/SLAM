#pragma once

#include "Frontend/VisualFrontend/PnP/PnPConfig.h"
#include "Frontend/VisualFrontend/PnP/PnPResult.h"

#include <opencv2/core.hpp>

#include <vector>

class IPnPAlgorithm
{
public:
    virtual ~IPnPAlgorithm() = default;

    virtual bool estimate(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distortionCoefficients,
        PnPResult& result) const = 0;
};
