#pragma once

#include "Estimation/Measurement/Measurement.h"
#include "Frontend/Frontend.h"
#include "Frontend/VisualFrontend/Pipeline/VisualData.h"

#include "Frontend/VisualFrontend/FeatureDetectionDescription/IFeatureDetectorDescriptor.h"
#include "Frontend/VisualFrontend/FeatureMatchingTracking/IFeatureMatcher.h"
#include"Frontend/VisualFrontend/FeatureMatchingTracking/IFeatureTracker.h"
#include "Frontend/VisualFrontend/MotionBlurCheck/IMotionBlurChecker.h"
#include "Frontend/VisualFrontend/LightExposureCheck/IExposureCorrector.h"
#include "Frontend/VisualFrontend/CrossCheck/ICrossCheckStrategy.h"
#include "Frontend/VisualFrontend/DistanceDescriptorFilter/IDistanceFilter.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"

#include <memory>
#include <utility>


class VisualFrontend : public Frontend
{
public:
    virtual ~VisualFrontend() = default;

    virtual std::unique_ptr<Measurement> processVisual(
        const VisualData& data) = 0;

//FIELDS
protected:
    //CONSTRUCTOR: CALLED BY DERIVED CLASSES
    //DETERMINES ALL STRATGIES AS PER CONFIG
    explicit VisualFrontend(
        std::unique_ptr<IFeatureDetectorDescriptor> featureDetector,
        std::unique_ptr<IFeatureMatcher> featureMatcher,
        std::unique_ptr<IMotionBlurChecker> motionBlurChecker,
        std::unique_ptr<IExposureCorrector> exposureCorrector,
        std::unique_ptr<ICrossCheckStrategy> crossChecker,
        std::unique_ptr<IDistanceFilter> distanceFilter)
        : featureDetector_(std::move(featureDetector)),
          featureMatcher_(std::move(featureMatcher)),
          motionBlurChecker_(std::move(motionBlurChecker)),
          exposureCorrector_(std::move(exposureCorrector)),
          crossChecker_(std::move(crossChecker)),
          distanceFilter_(std::move(distanceFilter))
    {
    }

    std::unique_ptr<IFeatureDetectorDescriptor> featureDetector_; //ORB
    std::unique_ptr<IFeatureMatcher> featureMatcher_; //BRUTE FORCE
    std::unique_ptr<IFeatureTracker> featureTracker_; //OPTICAL FLOW
    std::unique_ptr<IMotionBlurChecker> motionBlurChecker_;
    std::unique_ptr<IExposureCorrector> exposureCorrector_;
    std::unique_ptr<ICrossCheckStrategy> crossChecker_;
    std::unique_ptr<IDistanceFilter> distanceFilter_;
  
    // - feature tracking: Optical Flow
    // - outlier rejection
    // - camera model handling
    
//FUNCTIONS
protected:
    //FRAME PREPROCESSING: GRAYSCALE, PYRAMID, UNDISTORT, MOTION BLUR, EXPOSURE
    bool isMotionBlurred(const cv::Mat& image) const; //LAPLACIAN VARIANCE
    bool isPoorlyExposed(const cv::Mat& image) const; //LIGHT EXPOSURE
    bool convertToGrayscale(const cv::Mat& input,cv::Mat& gray) const;

    bool undistortImage(
    const cv::Mat& input,
    cv::Mat& undistorted,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs) const;

    //DETECT AND COMPUTE FETAURES
    bool detectCompute(const cv::Mat& image,
	std::vector<cv::KeyPoint>& keypoints,
	cv::Mat& descriptors);

    //DESCRIPTOR MATCHING
    //BF INCLUDES THE KNN
    bool matchDescriptors(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        std::vector<cv::DMatch>& matches);

    bool knnMatchDescriptors(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        std::vector<cv::DMatch>& goodMatches,
        int k = 2);

    //LOWE RATIO TEST
    //NO STRTAGEY
    bool LoweRatioTest(const cv::Mat& descriptors1,const cv::Mat& descriptors2);

    //DISTANCE BASED FILTERING
    void filterByDescriptorDistance(
    const std::vector<cv::DMatch>& matches,
    std::vector<cv::DMatch>& filteredMatches,
    float maxDistance) const;

    //CROSS CHECK DESCRIPTORS
    bool crossCheckDescriptors(const cv::Mat& descriptors1,const cv::Mat& descriptors2);

    //RANSAC : 4 METHODS
    // ESSENTIAL MATRIX + RANSAC
    //CAMERA INTRINSICS REQUIRED
    bool estimateEssentialMatrixRANSAC(
        const std::vector<cv::KeyPoint>& prevKeypoints,
        const std::vector<cv::KeyPoint>& currKeypoints,
        const std::vector<cv::DMatch>& matches,
        const cv::Mat& cameraMatrix,
        cv::Mat& essentialMatrix,
        cv::Mat& inlierMask);

    //RECOVER POSE
    bool recoverRelativePose(
        const cv::Mat& essentialMatrix,
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        const cv::Mat& cameraMatrix,
        const cv::Mat& ransacInlierMask,
        cv::Mat& R,
        cv::Mat& t,
        cv::Mat& poseInlierMask);

    // FUNDAMENTAL MATRIX + RANSAC
    //CAMERA INTRINSICS NOT REQUIRED
    bool estimateFundamentalMatrixRANSAC(
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        cv::Mat& fundamentalMatrix,
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