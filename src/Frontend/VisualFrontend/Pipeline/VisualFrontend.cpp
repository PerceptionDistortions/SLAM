#include"Frontend/VisualFrontend/Pipeline/VisualFrontend.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/features2d.hpp>
#include <iostream>

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


bool VisualFrontend::undistortImage(
    const cv::Mat& input,
    cv::Mat& undistorted,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs) const
{
    if (input.empty()) return false;
    if (cameraMatrix.empty() || distCoeffs.empty()) return false;

    cv::undistort(
        input,
        undistorted,
        cameraMatrix,
        distCoeffs);

    return !undistorted.empty();
}


bool VisualFrontend::detectCompute(const cv::Mat& image,
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors)
{
    // 1. Validate input
    if (image.empty())
    {
        std::cerr << "VisualFrontend: Empty image passed "
                     "to detectCompute()."
                  << std::endl;
        return false;
    }

    // 2. Validate strategy
    if (!featureDetector_)
    {
        std::cerr << "VisualFrontend: Feature detector "
                     "strategy is not initialized."
                  << std::endl;
        return false;
    }

    // 3. Clear previous results
    keypoints.clear();
    descriptors.release();

    // 4. Detect keypoints and compute descriptors
    featureDetector_->detectAndCompute(
        image,
        keypoints,
        descriptors);

    // 5. Validate results
    if (keypoints.empty())
    {
        std::cerr << "VisualFrontend: No keypoints detected."<< std::endl;
        return false;
    }

    if (descriptors.empty())
    {
        std::cerr << "VisualFrontend: Descriptor computation "
                     "produced no descriptors."
                  << std::endl;
        keypoints.clear();
        return false;
    }

    // Each descriptor row must correspond to one keypoint.
    if (descriptors.rows !=
        static_cast<int>(keypoints.size()))
    {
        std::cerr << "VisualFrontend: Keypoint/descriptor "
                     "count mismatch."
                  << std::endl;
        keypoints.clear();
        descriptors.release();
        return false;
    }

    std::cout << "Features detected: "<<keypoints.size()<<std::endl;

    return true;
}

bool VisualFrontend::matchDescriptors(
    const cv::Mat& descriptors1,
    const cv::Mat& descriptors2,
    std::vector<cv::DMatch>& matches)
{
    matches.clear();

    if (!featureMatcher_)
    {
        std::cerr << "VisualFrontend: Feature matcher "
                     "is not initialized."
                  << std::endl;
        return false;
    }

    if (descriptors1.empty() || descriptors2.empty())
    {
        std::cerr << "VisualFrontend: Empty descriptors."
                  << std::endl;
        return false;
    }

    featureMatcher_->match(
        descriptors1,
        descriptors2,
        matches);

    if (matches.empty())
    {
        std::cerr << "VisualFrontend: No matches found."
                  << std::endl;
        return false;
    }

    return true;
}


bool VisualFrontend::knnMatchDescriptors(
    const cv::Mat& descriptors1,
    const cv::Mat& descriptors2,
    std::vector<cv::DMatch>& goodMatches,
    int k)
{
    goodMatches.clear();

    if (!featureMatcher_)
    {
        std::cerr << "VisualFrontend: Feature matcher "
                     "is not initialized."
                  << std::endl;
        return false;
    }

    if (descriptors1.empty() || descriptors2.empty())
    {
        std::cerr << "VisualFrontend: Empty descriptors."
                  << std::endl;
        return false;
    }

    if (k < 2)
    {
        std::cerr << "VisualFrontend: KNN matching requires "
                     "k >= 2 for the ratio test."
                  << std::endl;
        return false;
    }

    std::vector<std::vector<cv::DMatch>> knnMatches;

    try
    {
        featureMatcher_->knnMatch(
            descriptors1,
            descriptors2,
            knnMatches,
            k);
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "VisualFrontend: KNN matching failed or "
                     "is unsupported by the matcher: "
                  << e.what() << std::endl;
        return false;
    }

    constexpr float ratioThreshold = 0.75f;

    for (const auto& candidates : knnMatches)
    {
        if (candidates.size() < 2)
            continue;

        if (candidates[0].distance <
            ratioThreshold * candidates[1].distance)
        {
            goodMatches.push_back(candidates[0]);
        }
    }

    if (goodMatches.empty())
    {
        std::cerr << "VisualFrontend: KNN produced no matches "
                     "passing the ratio test."
                  << std::endl;
        return false;
    }

    std::cout << "knn matches:<<"<<goodMatches.size()<<std::endl;

    return true;
}

//ESSENTIAL MATRIX + RANSAC
bool VisualFrontend::estimateEssentialMatrixRANSAC(
    const std::vector<cv::KeyPoint>& prevKeypoints,
    const std::vector<cv::KeyPoint>& currKeypoints,
    const std::vector<cv::DMatch>& matches,
    const cv::Mat& cameraMatrix,
    cv::Mat& essentialMatrix,
    cv::Mat& inlierMask)
{
    essentialMatrix.release();
    inlierMask.release();

    if (matches.size() < 5 || cameraMatrix.empty()) {
        std::cerr << "Essential matrix: insufficient matches or calibration."
                  << std::endl;
        return false;
    }

    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    points1.reserve(matches.size());
    points2.reserve(matches.size());

    for (const auto& match : matches) {
        if (match.queryIdx < 0 ||
            match.trainIdx < 0 ||
            match.queryIdx >= static_cast<int>(prevKeypoints.size()) ||
            match.trainIdx >= static_cast<int>(currKeypoints.size())) {
            continue;
        }

        points1.push_back(
            prevKeypoints[match.queryIdx].pt);

        points2.push_back(
            currKeypoints[match.trainIdx].pt);
    }

    if (points1.size() < 5) {
        std::cerr << "Essential matrix: insufficient valid correspondences."
                  << std::endl;
        return false;
    }

    essentialMatrix = cv::findEssentialMat(
        points1,
        points2,
        cameraMatrix,
        cv::RANSAC,
        0.999,  // RANSAC confidence
        1.0,    // Maximum epipolar error in pixels
        inlierMask
    );

    if (essentialMatrix.empty() || inlierMask.empty()) {
        std::cerr << "Essential matrix estimation failed."
                  << std::endl;
        return false;
    }

    const int inlierCount = cv::countNonZero(inlierMask);
    const double inlierRatio =
        static_cast<double>(inlierCount) / points1.size();

    std::cout << "[E-RANSAC] Matches (inliers): " << inlierCount
              << ", inlier ratio: " << inlierRatio
              << std::endl;

    if (inlierCount < 5) {
        std::cerr << "Essential matrix: too few inliers."
                  << std::endl;
        return false;
    }

    return true;
}


bool VisualFrontend::recoverRelativePose(
    const cv::Mat& essentialMatrix,
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2,
    const cv::Mat& cameraMatrix,
    const cv::Mat& ransacInlierMask,
    cv::Mat& R,
    cv::Mat& t,
    cv::Mat& poseInlierMask)
{
    // Reset output parameters.
    R.release();
    t.release();
    poseInlierMask.release();

    // Validate inputs.
    if (essentialMatrix.empty() ||
        essentialMatrix.rows != 3 ||
        essentialMatrix.cols != 3 ||
        cameraMatrix.empty() ||
        points1.size() != points2.size() ||
        points1.size() < 5 ||
        ransacInlierMask.total() != points1.size())
    {
        std::cerr << "RecoverPose: invalid inputs."
                  << std::endl;
        return false;
    }

    // Preserve the original RANSAC mask.
    // recoverPose modifies the output mask in place.
    poseInlierMask = ransacInlierMask.clone();

    // Diagnostic experiment: test whether the default distance
    // threshold is rejecting otherwise valid points.
    constexpr double DISTANCE_THRESHOLD = 1e6;

    const int inlierCount = cv::recoverPose(
        essentialMatrix,
        points1,
        points2,
        cameraMatrix,
        R,
        t,
        DISTANCE_THRESHOLD,
        poseInlierMask
    );

    // Count the original RANSAC inliers.
    const int ransacInliers =cv::countNonZero(ransacInlierMask);

    // Count points surviving recoverPose's geometric checks.
    const int cheiralityInliers =cv::countNonZero(poseInlierMask);

    const double cheiralityRatio =
        ransacInliers > 0
            ? static_cast<double>(cheiralityInliers)
                  / ransacInliers
            : 0.0;

    // Print diagnostics BEFORE checking for failure.
    std::cout << "[RecoverPose]"
          << " surviving inliers=" << cheiralityInliers
          << ", ratio=" << cheiralityRatio
          << std::endl;

    // Validate pose recovery.
    if (R.empty() ||
        t.empty() ||
        inlierCount < 5)
    {
        R.release();
        t.release();
        poseInlierMask.release();

        std::cerr << "RecoverPose: pose recovery failed."
                  << std::endl;
        return false;
    }

    return true;
}
