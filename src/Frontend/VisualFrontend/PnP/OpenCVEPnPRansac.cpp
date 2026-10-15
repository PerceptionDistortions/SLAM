#include "Frontend/VisualFrontend/PnP/OpenCVEPnPRansac.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <iostream>

OpenCVEPnPRansac::OpenCVEPnPRansac()
    : config_()
{
}

OpenCVEPnPRansac::OpenCVEPnPRansac(const PnPConfig& config)
    : config_(config)
{
}

bool OpenCVEPnPRansac::estimate(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distortionCoefficients,
    PnPResult& result) const
{
    result = PnPResult{};
    result.candidateCount = static_cast<int>(objectPoints.size());

    // 1. Validate inputs.
    if (objectPoints.size() != imagePoints.size() ||
        objectPoints.size() <
            static_cast<std::size_t>(config_.minCorrespondences) ||
        cameraMatrix.rows != 3 ||
        cameraMatrix.cols != 3 ||
        cameraMatrix.empty() ||
        config_.minCorrespondences < 4 ||
        config_.iterationsCount <= 0 ||
        config_.reprojectionThreshold <= 0.0f ||
        config_.confidence <= 0.0 ||
        config_.confidence >= 1.0)
    {
        std::cerr << "[OpenCVEPnPRansac] Invalid input."
                  << std::endl;
        return false;
    }

    for (std::size_t i = 0; i < objectPoints.size(); ++i)
    {
        const auto& P = objectPoints[i];
        const auto& p = imagePoints[i];

        if (!std::isfinite(P.x) ||
            !std::isfinite(P.y) ||
            !std::isfinite(P.z) ||
            !std::isfinite(p.x) ||
            !std::isfinite(p.y))
        {
            std::cerr << "[OpenCVEPnPRansac] Non-finite point."
                      << std::endl;
            return false;
        }
    }

    // 2. EPnP inside RANSAC.
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inlierMatrix;

    const bool solved = cv::solvePnPRansac(
        objectPoints,
        imagePoints,
        cameraMatrix,
        distortionCoefficients,
        rvec,
        tvec,
        false,
        config_.iterationsCount,
        config_.reprojectionThreshold,
        config_.confidence,
        inlierMatrix,
        cv::SOLVEPNP_EPNP);

    if (!solved || inlierMatrix.empty())
    {
        std::cerr << "[OpenCVEPnPRansac] RANSAC failed."
                  << std::endl;
        return false;
    }

    // 3. Extract RANSAC inlier indices.
    cv::Mat indices;
    inlierMatrix.convertTo(indices, CV_32S);
    indices = indices.reshape(1, 1);

    result.inlierIndices.reserve(indices.total());

    for (int i = 0; i < static_cast<int>(indices.total()); ++i)
    {
        const int index = indices.at<int>(0, i);

        if (index < 0 ||
            index >= static_cast<int>(objectPoints.size()))
        {
            result = PnPResult{};
            return false;
        }

        result.inlierIndices.push_back(index);
    }

    result.inlierCount =
        static_cast<int>(result.inlierIndices.size());

    if (result.inlierCount < 4)
    {
        result = PnPResult{};
        return false;
    }

    result.inlierMask.assign(objectPoints.size(), 0);

    for (const int index : result.inlierIndices)
    {
        result.inlierMask[static_cast<std::size_t>(index)] = 1;
    }

    // 4. Optionally refine using only RANSAC inliers.
    cv::Mat refinedRvec = rvec.clone();
    cv::Mat refinedTvec = tvec.clone();

    if (config_.refineWithInliers)
    {
        std::vector<cv::Point3f> inlierObjectPoints;
        std::vector<cv::Point2f> inlierImagePoints;

        for (const int index : result.inlierIndices)
        {
            inlierObjectPoints.push_back(objectPoints[index]);
            inlierImagePoints.push_back(imagePoints[index]);
        }

        const bool refined = cv::solvePnP(
            inlierObjectPoints,
            inlierImagePoints,
            cameraMatrix,
            distortionCoefficients,
            refinedRvec,
            refinedTvec,
            true,
            cv::SOLVEPNP_ITERATIVE);

        if (!refined)
        {
            // Preserve the valid RANSAC estimate.
            refinedRvec = rvec.clone();
            refinedTvec = tvec.clone();
        }
    }

    // 5. Convert rotation vector to rotation matrix.
    cv::Mat R;
    cv::Rodrigues(refinedRvec, R);

    // 6. Compute final inlier reprojection error and positive depth.
    std::vector<cv::Point2f> projectedPoints;

    cv::projectPoints(
        objectPoints,
        refinedRvec,
        refinedTvec,
        cameraMatrix,
        distortionCoefficients,
        projectedPoints);

    cv::Mat R64;
    cv::Mat t64;

    R.convertTo(R64, CV_64F);
    refinedTvec.convertTo(t64, CV_64F);

    double totalError = 0.0;
    int positiveDepthCount = 0;

    for (const int index : result.inlierIndices)
    {
        const auto& observed = imagePoints[index];
        const auto& projected = projectedPoints[index];

        const double dx = observed.x - projected.x;
        const double dy = observed.y - projected.y;

        totalError += std::sqrt(dx * dx + dy * dy);

        const auto& P = objectPoints[index];

        const cv::Mat pointWorld =
            (cv::Mat_<double>(3, 1) << P.x, P.y, P.z);

        const cv::Mat pointCamera = R64 * pointWorld + t64;

        if (pointCamera.at<double>(2, 0) > 0.0)
        {
            ++positiveDepthCount;
        }
    }

    result.rotation = R64;
    result.translation = t64;
    result.rotationVector = refinedRvec.clone();

    result.meanReprojectionError =
        totalError / result.inlierCount;

    result.positiveDepthCount = positiveDepthCount;
    result.success = true;

    return true;
}