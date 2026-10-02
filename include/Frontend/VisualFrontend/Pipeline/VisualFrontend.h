#pragma once

#include "Estimation/Measurement/Measurement.h"
#include"Frontend/Frontend.h"
#include "Frontend/VisualFrontend/Pipeline/VisualData.h"

#include <memory>

class VisualFrontend : public Frontend
{
public:
    virtual ~VisualFrontend() = default;

    virtual std::unique_ptr<Measurement> processVisual(
        const VisualData& data) = 0;

//FIELDS
protected:
    // - feature detection
    // - feature matching
    // - feature tracking
    // - outlier rejection
    // - camera model handling
    
//FUNCTIONS
protected:
    //FRAME PREPROCESSING: GRAYSCALE, PYRAMID, UNDISTORT, MOTION BLUR, EXPOSURE
    bool isMotionBlurred(const cv::Mat& image) const; //LAPLACIAN VARIANCE
    bool isPoorlyExposed(const cv::Mat& image) const; //LIGHT EXPOSURE
    bool preprocessImage(cv::Mat& image);

    //DETECT AND COMPUTE FETAURES
    bool detectCompute(const cv::Mat& image,
	std::vector<cv::KeyPoint>& keypoints,
	cv::Mat& descriptors);

    //DESCRIPTOR MATCHING
    bool matchDescriptors(const cv::Mat& descriptors1,const cv::Mat& descriptors2);

    //KNN MATCHING
    bool knnMatchDescriptors(const cv::Mat& descriptors1,const cv::Mat& descriptors2);

    //LOWE RATIO TEST
    //NO STRTAGEY
    bool LoweRatioTest(const cv::Mat& descriptors1,const cv::Mat& descriptors2);

    //CROSS CHECK DESCRIPTORS
    bool crossCheckDescriptors(const cv::Mat& descriptors1,const cv::Mat& descriptors2);

    //RANSAC : 4 METHODS
    // ESSENTIAL MATRIX + RANSAC
    bool estimateEssentialMatrixRANSAC(
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        cv::Mat& essentialMatrix,
        cv::Mat& inlierMask);

    // FUNDAMENTAL MATRIX + RANSAC
    bool estimateFundamentalMatrixRANSAC(
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        cv::Mat& fundamentalMatrix,
        cv::Mat& inlierMask);

    // HOMOGRAPHY + RANSAC
    bool estimateHomographyRANSAC(
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        cv::Mat& homography,
        cv::Mat& inlierMask);

    // PnP + RANSAC
    bool estimatePnPRANSAC(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        cv::Mat& rvec,
        cv::Mat& tvec,
        cv::Mat& inlierMask);

    //MOTION GATING CHECK BETWEEN FRAMES
    //IF THE FRAME DISPLACMENT REALISTIC?
};