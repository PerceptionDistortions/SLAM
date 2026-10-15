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
    std::unique_ptr<IMotionGating> motionGating,
    std::unique_ptr<IPnPAlgorithm> pnpRansac,
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
          std::move(motionGating),
          std::move(pnpRansac),
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



    //DETECT COMPUTE: USE RESULTS IN KEYPOINTS AND DESCRIPTORS
    //USE PREPROCSSED IMAGE 
    if(!detectCompute(processedImage, keypoints_, descriptors_))
    {
        std::cerr << "Feature detection and description failed."<< std::endl;
        return nullptr;
    }



    //FIRST FRAME HANDLING
    //OBSERVATIONS AND TRACKS
    //PREV FRAME, PREV KEYPOINTS, PREV DESCRIPTORS
    if(!hasPrevFrame_)
    {
        const std::vector<FeatureObservation> currentObservations =
        createObservations(currentFrameId, cameraId_, keypoints_);

        // Initialize tentative tracks for the first frame.
        trackManager_->update(currentObservations, {}, {});

        prevKeypoints_ = keypoints_;
        prevDescriptors_ = descriptors_.clone();
        prevFrameId_ = currentFrameId;
        hasPrevFrame_ = true;
        return nullptr;
    }



    //FEATURE MATCHING
    //IF NORMAL BF+LOWE, ELSE LOWE REQUIRED
    //STORE RESULTS IN DMATCH DS
    if(!knnMatchDescriptors(prevDescriptors_, descriptors_,matches_))
    {
        std::cerr << "Feature matching failed."<< std::endl;
        return nullptr;
    }



    //FURTHER DESCRIPTOR FILTERING
    //IMPLEMENT LATER: SKIP FOR NOW
    //OPTIONAL: CROSS CHECK
    //OPTIONAL: DISTANCE FILTERING

    

    //PIXEL CORRESEPONDECES BETWEEN MATCHES
    //USES RESULTS FROM THE BF MATCHING: DMATCH
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

        const cv::Point2f& p1 =prevKeypoints_[match.queryIdx].pt;

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



    //DEFINE VARS REQUIRED FOR ESSENTIAL+RECOVER POSE
    cv::Mat essentialMatrix; //RESULT: GEOMETRY BETWEEN 2 FRAMES
    cv::Mat inlierMask; //USED TO VERIFY WHICH OF MATCHES ARE ALIGNED WITH GEOMETRY

    //Recover relative pose
    cv::Mat R;
    cv::Mat t;
    cv::Mat poseInlierMask;

    //DEFINE VARS REQUIRED FOR PNP
    PnPResult pnpResult;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    //COUNT LANDMARKS AND DECIDE BETWEEN E AND PNP
    std::cout<<"Landmarks count: "<<mapManager_.getLandmarkCount()<<std::endl;// SELECT POSE ESTIMATION METHOD BASED ON MAP LANDMARK COUNT
    const bool usePnP = mapManager_.getLandmarkCount() >= 6;

    if (usePnP)
    {
        objectPoints.clear();
        imagePoints.clear();

        std::vector<std::size_t> pnpMatchIndices;
        pnpMatchIndices.reserve(validMatches.size());
        objectPoints.reserve(validMatches.size());
        imagePoints.reserve(validMatches.size());

        const auto trackIds = trackManager_->getTrackIds();

        for (std::size_t matchIndex = 0; matchIndex < validMatches.size(); ++matchIndex)
        {
            const auto& match = validMatches[matchIndex];

            if (match.queryIdx < 0 || match.trainIdx < 0 ||
                match.queryIdx >= static_cast<int>(prevKeypoints_.size()) ||
                match.trainIdx >= static_cast<int>(keypoints_.size()))
                continue;

            IFeatureTrack* matchedTrack = nullptr;

            for (const TrackId trackId : trackIds)
            {
                IFeatureTrack* track = trackManager_->getTrack(trackId);
                if (track == nullptr) continue;

                const auto& observations = track->getObservations();

                for (const auto& observation : observations)
                {
                    if (observation.frameId == prevFrameId_ &&
                        observation.featureId == static_cast<FeatureId>(match.queryIdx) &&
                        observation.cameraId == cameraId_)
                    {
                        matchedTrack = track;
                        break;
                    }
                }

                if (matchedTrack != nullptr) break;
            }

            if (matchedTrack == nullptr) continue;

            const auto landmarkId = matchedTrack->getLandmarkId();
            if (!landmarkId.has_value()) continue;

            const auto landmark = mapManager_.getLandmark(*landmarkId);
            if (!landmark || !landmark->isValid()) continue;

            const Eigen::Vector3d& position = landmark->getPosition();

            objectPoints.emplace_back(static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()));
            imagePoints.push_back(keypoints_[match.trainIdx].pt);
            pnpMatchIndices.push_back(matchIndex);
        }

        if (!pnpRansac_ || objectPoints.size() < 6 ||
            objectPoints.size() != imagePoints.size() ||
            objectPoints.size() != pnpMatchIndices.size() ||
            !pnpRansac_->estimate(objectPoints, imagePoints, cameraMatrix_, distCoeffs_, pnpResult) ||
            !pnpResult.success)
        {
            std::cerr << "PnP pose estimation failed." << std::endl;
            return nullptr;
        }

        // PnP estimates the world-to-camera pose: X_camera = R * X_world + t.
        R = pnpResult.rotation;
        t = pnpResult.translation;

        inlierMask = cv::Mat::zeros(1, static_cast<int>(validMatches.size()), CV_8UC1);

        for (std::size_t i = 0; i < pnpResult.inlierMask.size() && i < pnpMatchIndices.size(); ++i)
        {
            if (pnpResult.inlierMask[i] != 0)
                inlierMask.at<std::uint8_t>(0, static_cast<int>(pnpMatchIndices[i])) = 1;
        }

        poseInlierMask = inlierMask.clone();
    }
    else
    {
        if (!estimateEssentialMatrixRANSAC(prevKeypoints_,keypoints_,validMatches,cameraMatrix_,essentialMatrix,inlierMask))
        {
            std::cerr << "Essential matrix estimation failed."<< std::endl;
            return nullptr;
        }

        if (!recoverRelativePose(essentialMatrix,points1,points2,cameraMatrix_,inlierMask,R,t,poseInlierMask))
        {
            std::cerr << "Relative pose recovery failed."<< std::endl;
            return nullptr;
        }

        //DETERMINE THE NEW FRAME POSE
    }

    //PRINT FRAME POSE
    //E+RECOVER POSE GIVES RELATIVE POSE BUT PNP GIVES ABSOLUTE POSE

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
            << std::endl;

    //MOTION GATING: GEOMETRIC VERIFICATION: PNP AND ESSENTIAL MATRIX
    //VALID FEATURES BASED ON CAMERA MOTION
    //DOES NOT AFFECT FEATURES OR TRACKS
    //ONLY DISCARDS FEATURES IF FAILS
    MotionGatingInput gatingInput;
    gatingInput.candidateMatchCount = static_cast<int>(validMatches.size());

    cv::Mat ransacMask = inlierMask.reshape(1, 1);

    if (ransacMask.type() != CV_8UC1 || ransacMask.total() != validMatches.size())
    {
        std::cerr << "Invalid RANSAC inlier mask." << std::endl;
        return nullptr;
    }

    gatingInput.ransacInlierCount = cv::countNonZero(ransacMask);
    gatingInput.rotation = R;
    gatingInput.translation = t;

    if (usePnP)
    {
        gatingInput.method = MotionEstimationType::PnP;
        gatingInput.meanReprojectionError = pnpResult.meanReprojectionError;
        gatingInput.positiveDepthCount = pnpResult.positiveDepthCount;
        gatingInput.recoverPoseInlierCount = 0;
    }
    else
    {
        gatingInput.method = MotionEstimationType::EssentialMatrix;
        gatingInput.recoverPoseInlierCount = cv::countNonZero(finalMask);
    }

    const MotionGatingResult gatingResult = motionGating_->evaluate(gatingInput);

    if (!gatingResult.accepted)
    {
        std::cerr << "[MotionGating] Rejected pose. Reason: "
                << static_cast<int>(gatingResult.failure)
                << " | RANSAC ratio: " << gatingResult.ransacInlierRatio
                << " | Cheirality ratio: " << gatingResult.cheiralityRatio
                << " | Rotation: " << gatingResult.rotationDegrees
                << " degrees" << std::endl;

        prevKeypoints_ = keypoints_;
        prevDescriptors_ = descriptors_.clone();
        prevFrameId_ = currentFrameId;

        return nullptr;
    }
    else
    {
        std::cerr << "Motion gating passed." << std::endl;
    }

    //POSE ONLY REFINEMENT: MINIMIZE THE REPROJECTION ERROR
    //AT LEAST 20 CORRESPONCECES BETWEN FEATURES AND LANDMARKS

    //PARALLAX AND BASELINE CHECKS
    //DOES NOT DISCARD A FRAME OR A KEYFRAME
    //DOES NOT DEMOTE A TRACK IF IT FAILS
    //MARK TRIANGULATION TRACKS
    for (const TrackId trackId : trackManager_->getTrackIds())
    {
        IFeatureTrack* track = trackManager_->getTrack(trackId);
        if (track == nullptr)
            continue;

        const auto& observations = track->getObservations();

        // Find observations belonging to:
        // prevFrameId_ and currentFrameId.

        // Skip if either observation is missing.

        // Convert both pixel coordinates to bearing rays.
        // Check parallax.
        // Check baseline.
        // If both pass, add this track to triangulation candidates.
    }


    //TRIANGULATION: POTENTIAL LANDMARKS CREATED

    //UPDATE TRACKS

    //REPROJECTION ERROR FOR TRIANGULATED TRACKS: QUALITY

    //DECIDE LANDMARKS


    //UPDATE PREV FRAME DATA
    prevKeypoints_ = keypoints_;
    prevDescriptors_ = descriptors_.clone();
    prevFrameId_ = currentFrameId;

    return nullptr;
}

void MonocularVisualFrontend::shutdown(){
    
}


bool MonocularVisualFrontend::preprocessImage(const MonocularFrame& frame,
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
