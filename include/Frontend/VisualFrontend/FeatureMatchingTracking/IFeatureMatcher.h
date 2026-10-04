#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <vector>

class IFeatureMatcher
{
public:
    virtual ~IFeatureMatcher() = default;

    virtual void match(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        std::vector<cv::DMatch>& matches) = 0;

    virtual void knnMatch(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        std::vector<std::vector<cv::DMatch>>& matches,
        int k = 2) = 0;
};