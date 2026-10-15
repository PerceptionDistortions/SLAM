#include "Frontend/VisualFrontend/MotionGating/MonocularMotionGating.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool convertToDouble(
    const cv::Mat& input,
    cv::Mat& output)
{
    if (input.empty())
    {
        return false;
    }

    try
    {
        input.convertTo(output, CV_64F);
    }
    catch (const cv::Exception&)
    {
        return false;
    }

    return cv::checkRange(output);
}
}

MonocularMotionGating::MonocularMotionGating(
    const MonocularMotionGatingConfig& config)
    : config_(config)
{
}

MotionGatingResult MonocularMotionGating::evaluate(
    const MotionGatingInput& input) const
{
    MotionGatingResult result;

    // --------------------------------------------------
    // 1. COMMON VALIDATION: CANDIDATE MATCHES
    // --------------------------------------------------
    if (input.candidateMatchCount < config_.minCandidateMatches)
    {
        result.failure =
            MotionGatingFailure::InsufficientMatches;
        return result;
    }

    // --------------------------------------------------
    // 2. COMMON VALIDATION: RANSAC INLIERS
    // --------------------------------------------------
    if (input.ransacInlierCount < 0 ||
        input.ransacInlierCount > input.candidateMatchCount)
    {
        result.failure = MotionGatingFailure::InvalidPose;
        return result;
    }

    if (input.ransacInlierCount < config_.minRansacInliers)
    {
        result.failure =
            MotionGatingFailure::InsufficientRansacInliers;
        return result;
    }

    result.ransacInlierRatio =
        static_cast<double>(input.ransacInlierCount) /
        static_cast<double>(input.candidateMatchCount);

    if (result.ransacInlierRatio < config_.minRansacInlierRatio)
    {
        result.failure =
            MotionGatingFailure::LowRansacInlierRatio;
        return result;
    }

    // --------------------------------------------------
    // 3. COMMON VALIDATION: ROTATION
    // --------------------------------------------------
    if (!isValidRotation(input.rotation))
    {
        result.failure = MotionGatingFailure::InvalidRotation;
        return result;
    }

    result.rotationDegrees =
        computeRotationDegrees(input.rotation);

    // --------------------------------------------------
    // 4. METHOD-SPECIFIC VALIDATION
    // --------------------------------------------------
    switch (input.method)
    {
        // ==============================================
        // ESSENTIAL MATRIX + RECOVER POSE
        // ==============================================
        case MotionEstimationType::EssentialMatrix:
        {
            if (input.recoverPoseInlierCount < 0 ||
                input.recoverPoseInlierCount >
                    input.ransacInlierCount)
            {
                result.failure = MotionGatingFailure::InvalidPose;
                return result;
            }

            // Minimum inliers surviving recoverPose.
            if (input.recoverPoseInlierCount <
                config_.minRecoverPoseInliers)
            {
                result.failure =
                    MotionGatingFailure::InsufficientPoseInliers;
                return result;
            }

            // Cheirality ratio.
            result.cheiralityRatio =
                static_cast<double>(input.recoverPoseInlierCount) /
                static_cast<double>(input.ransacInlierCount);

            if (result.cheiralityRatio <
                config_.minCheiralityRatio)
            {
                result.failure =
                    MotionGatingFailure::LowCheiralityRatio;
                return result;
            }

            // Relative translation must have a valid direction.
            if (!isValidTranslation(input.translation))
            {
                result.failure =
                    MotionGatingFailure::InvalidTranslation;
                return result;
            }

            // Reject implausibly large relative rotations.
            if (result.rotationDegrees >
                config_.maxRotationDegrees)
            {
                result.failure =
                    MotionGatingFailure::ExcessiveRotation;
                return result;
            }

            break;
        }

        // ==============================================
        // PNP + RANSAC
        // ==============================================
        case MotionEstimationType::PnP:
        {
            // PnP uses 3D landmarks and 2D image observations.
            // Validate reprojection quality instead of recoverPose
            // inlier count and cheirality ratio.

            if (!std::isfinite(input.meanReprojectionError) ||
                input.meanReprojectionError < 0.0 ||
                input.meanReprojectionError >
                    config_.maxPnpMeanReprojectionError)
            {
                result.failure = MotionGatingFailure::InvalidPose;
                return result;
            }

            // Validate positive-depth PnP inliers.
            if (input.positiveDepthCount <
                    config_.minPnpPositiveDepthCount ||
                input.positiveDepthCount >
                    input.ransacInlierCount)
            {
                result.failure = MotionGatingFailure::InvalidPose;
                return result;
            }

            // Do not apply recoverPose-specific checks here.
            // Do not gate the absolute map-to-camera rotation
            // using maxRotationDegrees.
            // Do not apply the relative-translation norm check.

            break;
        }

        default:
        {
            result.failure = MotionGatingFailure::InvalidPose;
            return result;
        }
    }

    // --------------------------------------------------
    // 5. ACCEPT POSE
    // --------------------------------------------------
    result.accepted = true;
    result.failure = MotionGatingFailure::None;

    return result;
}

bool MonocularMotionGating::isValidRotation(
    const cv::Mat& rotation) const
{
    if (rotation.rows != 3 || rotation.cols != 3)
    {
        return false;
    }

    cv::Mat R;
    if (!convertToDouble(rotation, R))
    {
        return false;
    }

    const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    const cv::Mat orthogonalityError =
        R.t() * R - identity;

    const double error = cv::norm(
        orthogonalityError, cv::NORM_INF);

    const double determinant = cv::determinant(R);

    return error <=
               config_.rotationOrthogonalityTolerance &&
           std::abs(determinant - 1.0) <=
               config_.rotationOrthogonalityTolerance;
}

bool MonocularMotionGating::isValidTranslation(
    const cv::Mat& translation) const
{
    if (translation.empty() ||
        translation.total() != 3)
    {
        return false;
    }

    cv::Mat t;
    if (!convertToDouble(translation, t))
    {
        return false;
    }

    return cv::norm(t, cv::NORM_L2) >
           config_.minTranslationNorm;
}

double MonocularMotionGating::computeRotationDegrees(
    const cv::Mat& rotation) const
{
    cv::Mat R;
    rotation.convertTo(R, CV_64F);

    const double cosine =
        (cv::trace(R)[0] - 1.0) * 0.5;

    const double angleRadians =
        std::acos(std::clamp(cosine, -1.0, 1.0));

    return angleRadians * 180.0 / kPi;
}
