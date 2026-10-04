#include "Frontend/VisualFrontend/FeatureMatchingTracking/BruteForceMatcher.h"

BruteForceMatcher::BruteForceMatcher(int normType)
    : matcher_(normType, false)
{
}

void BruteForceMatcher::match(
    const cv::Mat& descriptors1,
    const cv::Mat& descriptors2,
    std::vector<cv::DMatch>& matches)
{
    matches.clear();

    if (descriptors1.empty() || descriptors2.empty())
    {
        return;
    }

    matcher_.match(
        descriptors1,
        descriptors2,
        matches);
}

void BruteForceMatcher::knnMatch(
    const cv::Mat& descriptors1,
    const cv::Mat& descriptors2,
    std::vector<std::vector<cv::DMatch>>& matches,
    int k)
{
    matches.clear();

    if (descriptors1.empty() ||
        descriptors2.empty() ||
        k <= 0)
    {
        return;
    }

    matcher_.knnMatch(
        descriptors1,
        descriptors2,
        matches,
        k);
}