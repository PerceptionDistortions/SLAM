#include "Frontend/VisualFrontend/DistanceDescriptorFilter/FixedDistanceFilter.h"

FixedDistanceFilter::FixedDistanceFilter(float maxDistance)
    : maxDistance_(maxDistance)
{
}

void FixedDistanceFilter::filter(
    const std::vector<cv::DMatch>& matches,
    std::vector<cv::DMatch>& filteredMatches) const
{
    filteredMatches.clear();

    for (const auto& match : matches)
    {
        if (match.distance <= maxDistance_)
        {
            filteredMatches.push_back(match);
        }
    }
}