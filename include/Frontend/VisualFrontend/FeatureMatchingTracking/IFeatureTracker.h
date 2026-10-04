#pragma once

#include <opencv2/core.hpp>

#include <vector>

class IFeatureTracker
{
public:
    virtual ~IFeatureTracker() = default;

    virtual bool track(
        const cv::Mat& image1,
        const cv::Mat& image2,
        const std::vector<cv::Point2f>& points1,
        std::vector<cv::Point2f>& points2,
        std::vector<unsigned char>& status,
        std::vector<float>& errors) = 0;
};