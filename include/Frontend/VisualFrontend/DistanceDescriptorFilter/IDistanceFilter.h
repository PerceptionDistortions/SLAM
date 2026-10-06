#pragma once

#include <opencv2/features2d.hpp>
#include <vector>

class IDistanceFilter
{
public:
    virtual ~IDistanceFilter() = default;

    virtual void filter(
        const std::vector<cv::DMatch>& matches,
        std::vector<cv::DMatch>& filteredMatches) const = 0;
};