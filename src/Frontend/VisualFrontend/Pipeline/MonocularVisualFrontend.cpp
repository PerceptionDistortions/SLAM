#include"Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"

#include <utility>
#include<iostream>

MonocularVisualFrontend::MonocularVisualFrontend(
    const FrontendConfig& config,
    const CameraCalibration& cameraCalibration,
    std::unique_ptr<IFeatureDetectorDescriptor> featureDetector,
    std::unique_ptr<IFeatureMatcher> featureMatcher,
    std::unique_ptr<IMotionBlurChecker> motionBlurChecker,
    std::unique_ptr<IExposureCorrector> exposureCorrector,
    std::unique_ptr<ICrossCheckStrategy> crossChecker,
    std::unique_ptr<IDistanceFilter> distanceFilter)
    : VisualFrontend(
          std::move(featureDetector),
          std::move(featureMatcher),
          std::move(motionBlurChecker),
          std::move(exposureCorrector),
          std::move(crossChecker),
          std::move(distanceFilter)),
      config_(config),cameraCalibration_(cameraCalibration)
{
    initializeCalibration();
}

//OVERRIDEN METHODS
bool MonocularVisualFrontend::init(){
    return true;
}

std::unique_ptr<Measurement> MonocularVisualFrontend::processVisual(const VisualData& data){

    //CHECK IF PROVIDED DATA VALID
    const MonocularFrame* frame = std::get_if<MonocularFrame>(&data);
    if (frame == nullptr)
    {
        std::cerr << "MonocularVisualFrontend: ""VisualData does not contain MonocularFrame."
                  << std::endl;
        return nullptr;
    }

    //PREPROCESS IMAGE: UNDISTORT, GRAYSCALE, MOTION BLUR, EXPOSURE
    cv::Mat processedImage;
    if (!preprocessImage(*frame, processedImage))
    {
        std::cerr << "MonocularVisualFrontend: "
                     "Image preprocessing failed."
                  << std::endl;

        return nullptr;
    }

    //DETECT COMPUTE: ORB, SUPERPOINT, ETC.
    //STORES DATA IN CURRENT FRAME KEYPOINTS AND DESCRIPTORS
    //USE PREPROCESSED IMAGE, NOT ORIGINAL IMAGE
    if(!detectCompute(processedImage, keypoints_, descriptors_))
    {
        std::cerr << "MonocularVisualFrontend: "
                     "Feature detection and description failed."<< std::endl;
        return nullptr;
    }

    //IF FIRST FRAME, DO NOT COMPUTE FURTHER
    if(!hasPrevFrame_)
    {
        prevKeypoints_ = keypoints_;
        prevDescriptors_ = descriptors_.clone();
        hasPrevFrame_ = true;
        return nullptr;
    }

    //FEATURE MATCHING
    //IF NORMAL MATCH, USE LOWE
    //IF KNN MATCH, NO LOWE
    if(!knnMatchDescriptors(prevDescriptors_, descriptors_,matches_))
    {
        std::cerr << "MonocularVisualFrontend: ""Feature matching failed."
                  << std::endl;

        return nullptr;
    }

    //OPTIONAL: CROSS CHECK
    //OPTIONAL: DISTANCE FILTERING

    //MANDATORY: DECISION BETWEEN PNP AND ESSENTIAL MATRIX

    //BUILD MATCHED POINTS CORRESPONDENCES
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    points1.reserve(matches_.size());
    points2.reserve(matches_.size());

    for (const auto& match : matches_)
    {
        if (match.queryIdx < 0 ||match.trainIdx < 0 ||match.queryIdx >=
                static_cast<int>(prevKeypoints_.size()) ||
            match.trainIdx >=static_cast<int>(keypoints_.size()))
        {
            continue;
        }

        points1.push_back(prevKeypoints_[match.queryIdx].pt);
        points2.push_back(keypoints_[match.trainIdx].pt);
    }

    //ESSENTIAL MATRIX RANSAC
    cv::Mat essentialMatrix;
    cv::Mat inlierMask;

    if (!estimateEssentialMatrixRANSAC(
            prevKeypoints_,
            keypoints_,
            matches_,
            cameraMatrix_,
            essentialMatrix,
            inlierMask))
    {
        std::cerr << "MonocularVisualFrontend: "
                    "Essential matrix estimation failed."
                << std::endl;
        return nullptr;
    }

    //RECOVER RELATIVE POSE
    cv::Mat R;
    cv::Mat t;
    cv::Mat poseInlierMask;

    if (!recoverRelativePose(
            essentialMatrix,
            points1,
            points2,
            cameraMatrix_,
            inlierMask,
            R,
            t,
            poseInlierMask))
    {
        std::cerr << "MonocularVisualFrontend: "
                    "Relative pose recovery failed."
                << std::endl;
        return nullptr;
    }

    //PRINT THE RECOVERED POSE
    // std::cout << "[MonocularVisualFrontend] Relative pose recovered."
    //           << "\nRotation:\n" << R
    //           << "\nTranslation direction:\n" << t
    //           << std::endl;

    //UPDATE PREV FRAME DATA
    prevKeypoints_ = keypoints_;
    prevDescriptors_ = descriptors_.clone();

    return nullptr;
}

void MonocularVisualFrontend::shutdown(){
    
}


bool MonocularVisualFrontend::preprocessImage(
    const MonocularFrame& frame,
    cv::Mat& processedImage)
{
    processedImage.release();

    // 1. Validate input
    if (frame.camera.image.empty())
    {
        std::cerr << "MonocularVisualFrontend: "
                     "Empty input image."
                  << std::endl;
        return false;
    }

    // 2. Undistort
    cv::Mat undistortedImage;

    if (!undistortImage(
            frame.camera.image,
            undistortedImage,
            cameraMatrix_,
            distCoeffs_))
    {
        std::cerr << "MonocularVisualFrontend: "
                     "Failed to undistort image."
                  << std::endl;
        return false;
    }

    // 3. Convert to grayscale directly into output
    if (!convertToGrayscale(
            undistortedImage,
            processedImage))
    {
        std::cerr << "MonocularVisualFrontend: "
                     "Failed to convert image to grayscale."
                  << std::endl;
        return false;
    }

    //LIGHT EXPOSURE CHECK

    //MOTION BLUE CHECK

    return true;
}


void MonocularVisualFrontend::estimateMotion()
{
    // Motion estimation
}

void MonocularVisualFrontend::validateMotion()
{
    // Motion validation
}

void MonocularVisualFrontend::triangulate()
{
    // Landmark triangulation
}

void MonocularVisualFrontend::manageLandmarks()
{
    // Landmark management
}

void MonocularVisualFrontend::manageKeyframes()
{
    // Keyframe management
}

void MonocularVisualFrontend::initializeCalibration()
{
    // --------------------------------------------------
    // CAMERA INTRINSIC MATRIX
    //
    //       [ fx   0  cx ]
    // K  =  [  0  fy  cy ]
    //       [  0   0   1 ]
    // --------------------------------------------------

    cameraMatrix_ = (cv::Mat_<double>(3, 3) <<
        cameraCalibration_.fx, 0.0, cameraCalibration_.cx,
        0.0, cameraCalibration_.fy, cameraCalibration_.cy,
        0.0, 0.0, 1.0
    );

    // --------------------------------------------------
    // DISTORTION COEFFICIENTS
    //
    // Typically:
    // [ k1, k2, p1, p2, k3 ]
    // --------------------------------------------------

    distCoeffs_ = cv::Mat(
        1,
        static_cast<int>(cameraCalibration_.distortion.size()),
        CV_64F
    );

    for (size_t i = 0;
         i < cameraCalibration_.distortion.size();
         ++i)
    {
        distCoeffs_.at<double>(0, static_cast<int>(i)) =
            cameraCalibration_.distortion[i];
    }
}