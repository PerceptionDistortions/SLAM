#pragma once

#include <opencv2/features2d.hpp>
#include <vector>

class ICrossCheckStrategy
{
public:
    virtual ~ICrossCheckStrategy() = default;

    virtual void filter(
        const std::vector<cv::DMatch>& forwardMatches,
        const std::vector<cv::DMatch>& reverseMatches,
        std::vector<cv::DMatch>& filteredMatches) const = 0;
};