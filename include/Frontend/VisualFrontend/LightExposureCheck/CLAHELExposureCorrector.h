#pragma once

#include "IExposureCorrector.h"

class CLAHEExposureCorrector : public IExposureCorrector
{
public:
    CLAHEExposureCorrector(
        double clipLimit,
        int tileGridSize
    );

    cv::Mat correct(const cv::Mat& image) const override;

private:
    double clipLimit_;
    int tileGridSize_;
};