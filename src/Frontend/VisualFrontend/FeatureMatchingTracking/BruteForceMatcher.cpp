#include "Frontend/VisualFrontend/FeatureMatchingTracking/BruteForceMatcher.h"
#include <iostream>

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

    matcher_.match(descriptors1,descriptors2,matches);

    std::cout << "BruteForceMatcher: "
              << matches.size()
              << " matches found." << std::endl;
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

    matcher_.knnMatch(descriptors1,descriptors2,matches,k);

    std::size_t totalMatches = 0;

    for (const auto& candidates : matches)
        totalMatches += candidates.size();

    // std::cout << "BruteForceMatcher: "
    //           << matches.size() << " query descriptors, "
    //           << totalMatches << " candidate matches (k="
    //           << k << ")." << std::endl;
}