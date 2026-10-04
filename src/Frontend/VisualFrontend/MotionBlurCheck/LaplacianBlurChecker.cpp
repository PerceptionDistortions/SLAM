#include "LaplacianBlurChecker.h"

#include <opencv2/imgproc.hpp>

LaplacianBlurChecker::LaplacianBlurChecker(double threshold)
    : threshold_(threshold)
{
}

bool LaplacianBlurChecker::isBlurred(const cv::Mat& image) const
{
    if (image.empty())
        return true;

    cv::Mat gray;

    if (image.channels() == 1)
        gray = image;
    else
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat laplacian;

    cv::Laplacian(
        gray,
        laplacian,
        CV_64F
    );

    cv::Scalar mean;
    cv::Scalar stddev;

    cv::meanStdDev(laplacian, mean, stddev);

    const double variance = stddev[0] * stddev[0];

    return variance < threshold_;
}