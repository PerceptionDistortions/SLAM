#pragma once

#include <opencv2/core.hpp>

class IMotionBlurChecker
{
public:
    virtual ~IMotionBlurChecker() = default;

    virtual bool isBlurred(const cv::Mat& image) const = 0;
};