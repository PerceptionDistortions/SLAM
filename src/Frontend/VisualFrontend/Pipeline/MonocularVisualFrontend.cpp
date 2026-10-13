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
    std::unique_ptr<IDistanceFilter> distanceFilter,
    std::unique_ptr<ITrackManager> trackManager,
    MapManager& mapManager,
    std::string camId)
    : VisualFrontend(
          std::move(featureDetector),
          std::move(featureMatcher),
          std::move(motionBlurChecker),
          std::move(exposureCorrector),
          std::move(crossChecker),
          std::move(distanceFilter),
          std::move(trackManager),
          mapManager),
      config_(config),cameraCalibration_(cameraCalibration),
      cameraId_(std::move(camId))
{
    initializeCalibration();
}

//OVERRIDEN METHODS
bool MonocularVisualFrontend::init(){
    return true;
}

std::unique_ptr<Measurement> MonocularVisualFrontend::processVisual(const VisualData& data){

    //FRAME SHOULD NOT BE NULL
    //ASSIGN FRAME ID IF SUCCESS
    const MonocularFrame* frame = std::get_if<MonocularFrame>(&data);
    if (frame == nullptr)
    { 
        std::cerr << "MonocularFrame not valid."<< std::endl;
        return nullptr;
    }
    const FrameId currentFrameId = nextFrameId_++;

    //PREPROCESS IMAGE: UNDISTORT, GRAYSCALE, MOTION BLUR, EXPOSURE
    cv::Mat processedImage;
    if (!preprocessImage(*frame, processedImage))
    {
        std::cerr << "Image preprocessing failed."<< std::endl;
        return nullptr;
    }

    //DETECT COMPUTE: ORB, SUPERPOINT, ETC.
    //STORES DATA IN CURRENT FRAME KEYPOINTS AND DESCRIPTORS
    //USE PREPROCESSED IMAGE, NOT ORIGINAL IMAGE
    if(!detectCompute(processedImage, keypoints_, descriptors_))
    {
        std::cerr << "Feature detection and description failed."<< std::endl;
        return nullptr;
    }


    //IF FIRST FRAME: UPDATE PREV FRAME AND ABORT
    if(!hasPrevFrame_)
    {
        const std::vector<FeatureObservation> currentObservations =
        createObservations(currentFrameId, cameraId_, keypoints_);

        // Initialize tentative tracks for the first frame.
        trackManager_->update(currentObservations, {}, {});

        prevKeypoints_ = keypoints_;
        prevDescriptors_ = descriptors_.clone();
        hasPrevFrame_ = true;
        return nullptr;
    }

    //FEATURE MATCHING
    //IF NORMAL MATCH, USE LOWE
    //IF KNN MATCH, NO LOWE
    //KNN COVERS NORMAL MATCH TOO
    if(!knnMatchDescriptors(prevDescriptors_, descriptors_,matches_))
    {
        std::cerr << "Feature matching failed."<< std::endl;
        return nullptr;
    }

    //OPTIONAL: CROSS CHECK
    //OPTIONAL: DISTANCE FILTERING

    //MANDATORY: DECISION BETWEEN PNP AND ESSENTIAL MATRIX
    //FIRST CHECK IF ENOUGH LANDMARKS IN THE MAP
    std::cout<<"Landmarks count: "<<mapManager_.getLandmarkCount()<<std::endl;
    //TODO: BASED ON NUMBER OF ACTIVE LANDMARS, RANSAC MODE
    // if(mapManager_.getLandmarkCount()>=6){
        
    // }
    // else{

    // }

    //BUILD MATCHED POINTS CORRESPONDENCES
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    std::vector<cv::DMatch> validMatches;

    points1.reserve(matches_.size());
    points2.reserve(matches_.size());
    validMatches.reserve(matches_.size());

    for (const auto& match : matches_)
    {
        if (match.queryIdx < 0 ||match.trainIdx < 0 ||match.queryIdx >=static_cast<int>(prevKeypoints_.size()) ||
            match.trainIdx >=static_cast<int>(keypoints_.size())) continue;

        const cv::Point2f& p1 =
        prevKeypoints_[match.queryIdx].pt;

        const cv::Point2f& p2 = keypoints_[match.trainIdx].pt;

        if (!std::isfinite(p1.x) || !std::isfinite(p1.y) ||
            !std::isfinite(p2.x) || !std::isfinite(p2.y))
        {
            continue;
        }

        points1.push_back(p1);
        points2.push_back(p2);
        validMatches.push_back(match);
    }

    //ESSENTIAL MATRIX RANSAC
    //Separate checks for E and the geometric inlier
    cv::Mat essentialMatrix; //RESULT: GEOMETRY BETWEEN 2 FRAMES
    cv::Mat inlierMask; //USED TO VERIFY WHICH OF MATCHES ARE ALIGNED WITH GEOMETRY

    if (!estimateEssentialMatrixRANSAC(
            prevKeypoints_,
            keypoints_,
            validMatches,
            cameraMatrix_,
            essentialMatrix,
            inlierMask))
    {
        std::cerr << "Essential matrix estimation failed."<< std::endl;
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
        std::cerr << "Relative pose recovery failed."<< std::endl;
        return nullptr;
    }

    //PRINT THE RECOVERED POSE
    // std::cout << "[MonocularVisualFrontend] Relative pose recovered."
    //           << "\nRotation:\n" << R
    //           << "\nTranslation direction:\n" << t
    //           << std::endl;

    //UPDATE TENTATIVE AND ACTIVE TRACKS
    //masks should have dimensions predictable for indexing: RESHAPE METHOD
    cv::Mat finalMask = poseInlierMask.reshape(1, 1);

    if (finalMask.type() != CV_8UC1 ||finalMask.total() != validMatches.size())
    {
        std::cerr << "MonocularVisualFrontend: "
                    "Pose inlier mask does not match matches_."<< std::endl;
        return nullptr;
    }

    // Convert OpenCV's mask to the format expected by TrackManager.
    std::vector<std::uint8_t> geometricInlierMask(validMatches.size(), 0);
    for (std::size_t i = 0; i < validMatches.size(); ++i)
    {
        geometricInlierMask[i] =finalMask.at<std::uint8_t>(0, static_cast<int>(i)) != 0;
    }

    // Create observations for the current keypoints.
    // Use your actual frame ID and camera ID member names here.
    const std::vector<FeatureObservation> currentObservations =createObservations(
            currentFrameId,
            cameraId_,
            keypoints_);
    
    // Update existing tracks and create tentative tracks for
    // unmatched current-frame features.
    trackManager_->update(currentObservations,validMatches,geometricInlierMask);

    // Print track statistics.
    std::cout << "[Tracks]"
            << " Total: "
            << trackManager_->getTrackIds().size()
            << " | Tentative: "
            << trackManager_->getTrackIds(
                    TrackState::Tentative).size()
            << " | Active: "
            << trackManager_->getTrackIds(
                    TrackState::Active).size()
            << " | Triangulated: "
            << trackManager_->getTrackIds(
                    TrackState::Triangulated).size()
            << std::endl;

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