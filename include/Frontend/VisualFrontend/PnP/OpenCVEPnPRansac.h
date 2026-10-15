#pragma once

#include "Frontend/VisualFrontend/PnP/IPnPAlgorithm.h"

class OpenCVEPnPRansac final : public IPnPAlgorithm
{
public:
    OpenCVEPnPRansac();

    explicit OpenCVEPnPRansac(const PnPConfig& config);

    bool estimate(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distortionCoefficients,
        PnPResult& result) const override;

private:
    PnPConfig config_;
};