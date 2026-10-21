#pragma once

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include"Configuration/Configs/SystemConfig.h"
#include"Configuration/Configs/FrontendConfig.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"
#include "Frontend/VisualFrontend/Parallax/IParallaxChecker.h"
#include "Frontend/VisualFrontend/Baseline/IBaselineChecker.h"
#include"Frontend/VisualFrontend/Parallax/MonocularParallaxChecker.h"
#include"Frontend/VisualFrontend/Baseline/MonocularBaselineChecker.h"
#include"AliasesEnumsTypes/CameraPose.h"

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
    std::unique_ptr<IDistanceFilter> distanceFilter,
    std::unique_ptr<ITrackManager> trackManager,
    std::unique_ptr<IMotionGating> motionGating,
    std::unique_ptr<IPnPAlgorithm> pnpRansac,
    MapManager& mapManager,
    std::string camid
    );
    
    //DESTRUCTOR
    ~MonocularVisualFrontend() override = default;

    //OVERRIDEN METHODS
    std::unique_ptr<Measurement> processVisual(const VisualData& data) override;
    bool init() override;
    void shutdown() override;

private:
    //FRAME ID COUNTER
    FrameId nextFrameId_{0};
    FrameId prevFrameId_{0};
    
    // CAMERA CALIBRATION PASSED IN CONSRTRUCTOR, Camera Id
    std::string cameraId_;
    const FrontendConfig& config_;
    CameraCalibration cameraCalibration_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    void initializeCalibration();
    
    //PREVIOUS FRAME DATA
    std::vector<cv::KeyPoint> prevKeypoints_;
    cv::Mat prevDescriptors_;
    bool hasPrevFrame_ = false;
    CameraPose prevFramePose_;

    //CURRENT FRAME DATA
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    std::vector<cv::DMatch> matches_; //MATCH WITH PREVIOUS FRAME
    CameraPose currFramePose_;
    
    //PREPROCESSING, PROCESSING
    bool preprocessImage(const MonocularFrame& frame,cv::Mat& processedImage);

    //PARALLAX AND BASELINE CHECKS
    std::unique_ptr<IParallaxChecker> parallaxChecker_;
    std::unique_ptr<IBaselineChecker> baselineChecker_;

private:
//UTILITY FOR PROCESS VISUAL
    bool estimateFramePose(
        const std::vector<cv::Point2f>& points1,
        const std::vector<cv::Point2f>& points2,
        const std::vector<cv::DMatch>& validMatches,
        bool usePnP,
        cv::Mat& essentialMatrix,
        cv::Mat& inlierMask,
        cv::Mat& R,
        cv::Mat& t,
        cv::Mat& poseInlierMask,
        PnPResult& pnpResult);

    void buildValidCorrespondences(
        std::vector<cv::Point2f>& points1,
        std::vector<cv::Point2f>& points2,
        std::vector<cv::DMatch>& validMatches);

    std::vector<std::uint8_t> initialTrackUpdate(
        const cv::Mat& finalMask,
        const std::vector<cv::DMatch>& validMatches,
        FrameId currentFrameId);

    void triangulatedTrackUpdated(FrameId currentFrameId);

    bool evaluateMotionGating(
        const std::vector<cv::DMatch>& validMatches,
        const cv::Mat& inlierMask,
        const cv::Mat& finalMask,
        bool usePnP,
        const cv::Mat& R,
        const cv::Mat& t,
        const PnPResult& pnpResult,
        FrameId currentFrameId);

    std::vector<TrackId> parallaxBaselineCheck(FrameId currentFrameId,bool usePnP);

    //VALIDATE A TRNAGULATED POINT
    [[nodiscard]]
    bool validateTriangulatedPoint(
        const Eigen::Vector3d& pointWorld,
        const Eigen::Vector2d& previousPixel,
        const Eigen::Vector2d& currentPixel,
        const Eigen::Matrix4d& previousTcw,
        const Eigen::Matrix4d& currentTcw,
        const Eigen::Matrix3d& intrinsicMatrix) const;

    double minTriangulationReprojectionErrorPixels_{0.0};
    double maxTriangulationReprojectionErrorPixels_{3.0};
};