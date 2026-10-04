#pragma once

#include "Frontend/VisualFrontend/FeatureMatchingTracking/IFeatureMatcher.h"

class BruteForceMatcher : public IFeatureMatcher
{
public:
    explicit BruteForceMatcher(
        int normType = cv::NORM_HAMMING);

    ~BruteForceMatcher() override = default;

    void match(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        std::vector<cv::DMatch>& matches) override;

    void knnMatch(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        std::vector<std::vector<cv::DMatch>>& matches,
        int k = 2) override;

private:
    cv::BFMatcher matcher_;
};