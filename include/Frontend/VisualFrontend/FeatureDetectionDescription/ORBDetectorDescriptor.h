//ORIENTED FAST AND ROTATED BRIEF
//FAST-> KEYPOINTS
//BRIEF-> DESCRIPTORS ROTATION AWARE
//KEYPOINTS: PIXELS AND NEIGHBOR INTENSITY
//ROTATION INVARIANT
//SCALE INVARIANT: WEAK

#pragma once

#include <opencv2/features2d.hpp>
#include "IFeatureDetectorDescriptor.h"

class ORBDetectorDescriptor : public IFeatureDetectorDescriptor
{
public:
    ORBDetectorDescriptor(
        int nFeatures,
        double scaleFactor,
        int nLevels);

    void detectAndCompute(
        const cv::Mat& image,
        std::vector<cv::KeyPoint>& keypoints,
        cv::Mat& descriptors) override;

private:
    cv::Ptr<cv::ORB> orb_;
};