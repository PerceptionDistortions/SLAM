#pragma once

#include "IMotionBlurChecker.h"

class LaplacianBlurChecker : public IMotionBlurChecker
{
public:
    explicit LaplacianBlurChecker(double threshold);

    bool isBlurred(const cv::Mat& image) const override;

private:
    double threshold_;
};