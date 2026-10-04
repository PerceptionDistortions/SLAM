#include "Frontend/VisualFrontend/LightExposureCheck/CLAHELExposureCorrector.h"

#include <opencv2/imgproc.hpp>

CLAHEExposureCorrector::CLAHEExposureCorrector(
    double clipLimit,
    int tileGridSize)
    : clipLimit_(clipLimit),
      tileGridSize_(tileGridSize)
{
}

cv::Mat CLAHEExposureCorrector::correct(
    const cv::Mat& image) const
{
    if (image.empty())
        return image;

    cv::Mat gray;

    if (image.channels() == 1)
    {
        gray = image;
    }
    else
    {
        cv::cvtColor(
            image,
            gray,
            cv::COLOR_BGR2GRAY
        );
    }

    cv::Mat output;

    auto clahe = cv::createCLAHE(
        clipLimit_,
        cv::Size(tileGridSize_, tileGridSize_)
    );

    clahe->apply(gray, output);

    return output;
}