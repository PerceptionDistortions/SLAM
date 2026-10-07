#include"Frontend/VisualFrontend/Pipeline/VisualFrontend.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

bool VisualFrontend::convertToGrayscale(
    const cv::Mat& input,
    cv::Mat& gray) const
{
    if (input.empty()) return false;

    // Already grayscale
    if (input.channels() == 1)
    {
        gray = input;
        return true;
    }

    // Color image -> grayscale
    if (input.channels() == 3)
    {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
        return true;
    }

    // RGBA image -> grayscale
    if (input.channels() == 4)
    {
        cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
        return true;
    }

    return false;
}


bool VisualFrontend::undistortImage(const cv::Mat& input,cv::Mat& undistorted,
    const CameraCalibration& calibration) const
{
    if (input.empty())
        return false;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        calibration.fx, 0.0, calibration.cx,
        0.0, calibration.fy, calibration.cy,
        0.0, 0.0, 1.0);

    cv::Mat distCoeffs(
        calibration.distortion,
        true);

    cv::undistort(
        input,
        undistorted,
        cameraMatrix,
        distCoeffs);

    return !undistorted.empty();
}