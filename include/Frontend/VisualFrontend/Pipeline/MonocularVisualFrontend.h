#pragma once

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include"Configuration/Configs/SystemConfig.h"
#include"Configuration/Configs/FrontendConfig.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"


class MonocularVisualFrontend : public VisualFrontend
{
public:
    //CONSTRUCTOR: CALLED BY FACTORY
    MonocularVisualFrontend(
    const FrontendConfig& config,
    const CameraCalibration& cameraCalibration,
    std::unique_ptr<IFeatureDetectorDescriptor> featureDetector,
    std::unique_ptr<IFeatureMatcher> featureMatcher,
    std::unique_ptr<IMotionBlurChecker> motionBlurChecker,
    std::unique_ptr<IExposureCorrector> exposureCorrector,
    std::unique_ptr<ICrossCheckStrategy> crossChecker,
    std::unique_ptr<IDistanceFilter> distanceFilter
    );
    
    //DESTRUCTOR
    ~MonocularVisualFrontend() override = default;

    //OVERRIDEN METHODS
    std::unique_ptr<Measurement> processVisual(const VisualData& data) override;
    bool init() override;
    void shutdown() override;

private:
    // CAMERA CALIBRATION PASSED IN CONSRTRUCTOR
    const FrontendConfig& config_;
    CameraCalibration cameraCalibration_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    void initializeCalibration();
    
    //PREVIOUS FRAME DATA
    std::vector<cv::KeyPoint> prevKeypoints_;
    cv::Mat prevDescriptors_;
    bool hasPrevFrame_ = false;

    //CURRENT FRAME DATA
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    std::vector<cv::DMatch> matches_; //MATCH WITH PREVIOUS FRAME
    
    //PREPROCESSING, PROCESSING
    bool preprocessImage(const MonocularFrame& frame,cv::Mat& processedImage);

    void estimateMotion();

    void validateMotion();

    void triangulate();

    void manageLandmarks();

    void manageKeyframes();
};