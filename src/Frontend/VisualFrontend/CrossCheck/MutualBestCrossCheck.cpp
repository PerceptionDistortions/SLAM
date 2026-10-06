#include "Frontend/VisualFrontend/CrossCheck/MutualBestCrossCheck.h"

void MutualBestCrossCheck::filter(
    const std::vector<cv::DMatch>& forwardMatches,
    const std::vector<cv::DMatch>& reverseMatches,
    std::vector<cv::DMatch>& filteredMatches) const
{
    filteredMatches.clear();

    for (const auto& forwardMatch : forwardMatches)
    {
        for (const auto& reverseMatch : reverseMatches)
        {
            if (forwardMatch.queryIdx == reverseMatch.trainIdx &&
                forwardMatch.trainIdx == reverseMatch.queryIdx)
            {
                filteredMatches.push_back(forwardMatch);
                break;
            }
        }
    }
}