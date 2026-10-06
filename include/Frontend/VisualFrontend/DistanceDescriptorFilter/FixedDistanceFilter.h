#pragma once

#include "IDistanceFilter.h"

class FixedDistanceFilter : public IDistanceFilter
{
public:
    explicit FixedDistanceFilter(float maxDistance);

    ~FixedDistanceFilter() override = default;

    void filter(
        const std::vector<cv::DMatch>& matches,
        std::vector<cv::DMatch>& filteredMatches) const override;

private:
    float maxDistance_;
};