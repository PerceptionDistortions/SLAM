#include "Frontend/VisualFrontend/FeatureDetectionDescription/ORBDetectorDescriptor.h"

ORBDetectorDescriptor::ORBDetectorDescriptor(
    int nFeatures,
    double scaleFactor,
    int nLevels)
{
    orb_ = cv::ORB::create(
        nFeatures,
        scaleFactor,
        nLevels);
}

void ORBDetectorDescriptor::detectAndCompute(
    const cv::Mat& image,
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors)
{
    orb_->detectAndCompute(
        image,
        cv::noArray(),
        keypoints,
        descriptors);
}