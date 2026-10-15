
#pragma once

#include "IMotionGating.h"

//PUT IN CONFIG FILE
struct MonocularMotionGatingConfig
{
    int minCandidateMatches = 30;
    int minRansacInliers = 20;
    double minRansacInlierRatio = 0.25;

    int minRecoverPoseInliers = 15;
    double minCheiralityRatio = 0.35;

    // Set to 180.0 to effectively disable this check.
    double maxRotationDegrees = 90.0;

    double minTranslationNorm = 1e-8;
    double rotationOrthogonalityTolerance = 1e-3;

    double maxPnpMeanReprojectionError = 3.0;
    int minPnpPositiveDepthCount = 4;
};

class MonocularMotionGating final : public IMotionGating
{
public:
    explicit MonocularMotionGating(
        const MonocularMotionGatingConfig& config = {});

    [[nodiscard]]
    MotionGatingResult evaluate(
        const MotionGatingInput& input) const override;

private:
    MonocularMotionGatingConfig config_;

    [[nodiscard]]
    bool isValidRotation(
        const cv::Mat& rotation) const;

    [[nodiscard]]
    bool isValidTranslation(
        const cv::Mat& translation) const;

    [[nodiscard]]
    double computeRotationDegrees(
        const cv::Mat& rotation) const;
};
