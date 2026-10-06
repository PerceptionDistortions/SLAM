#pragma once

#include "ICrossCheckStrategy.h"

class MutualBestCrossCheck : public ICrossCheckStrategy
{
public:
    ~MutualBestCrossCheck() override = default;

    void filter(
        const std::vector<cv::DMatch>& forwardMatches,
        const std::vector<cv::DMatch>& reverseMatches,
        std::vector<cv::DMatch>& filteredMatches) const override;
};