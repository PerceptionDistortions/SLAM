#pragma once

#include <opencv2/core.hpp>

class IExposureCorrector
{
public:
    virtual ~IExposureCorrector() = default;

    virtual cv::Mat correct(const cv::Mat& image) const = 0;
};