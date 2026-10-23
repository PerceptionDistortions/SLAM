#include"Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"
#include "Frontend/VisualFrontend/Triangulation/MidpointTriangulator.h"

#include <utility>
#include<iostream>
#include <memory>
#include <Eigen/Geometry>
#include<unordered_set>

//CONSTRUCTOR
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
      cameraId_(std::move(camId)),
      parallaxChecker_(std::make_unique<MonocularParallaxChecker>(0.3)),
      baselineChecker_(std::make_unique<MonocularBaselineChecker>(0.01))
{
    initializeCalibration();
}

//OVERRIDEN METHODS
bool MonocularVisualFrontend::init(){
    return true;
}


std::unique_ptr<Measurement> MonocularVisualFrontend::processVisual(const VisualData& data){

    //FRAME NULL CHECK, ASSIGN FRAME ID
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


    //DETECT COMPUTE (USE PREPROCESSED IMAGE): KEYPOINTS, DESCRIPTORS
    if(!detectCompute(processedImage, keypoints_, descriptors_))
    {
        std::cerr << "Feature detection and description failed."<< std::endl;
        return nullptr;
    }


    //FIRST FRAME INIT: OBSERVATIONS, TRACK INIT, PREV FRAME
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

        //Pose of first frame
        prevFramePose_.worldFromCameraRotation = Eigen::Matrix3d::Identity();
        prevFramePose_.positionWorld = Eigen::Vector3d::Zero();
        prevFramePose_.valid = true;

        return nullptr;
    }


    //FEATURE MATCHING: STORE RESULT IN DMATCH
    //KNN, or NORMAL+LOWE IN BF
    if(!knnMatchDescriptors(prevDescriptors_, descriptors_,matches_))
    {
        std::cerr << "Feature matching failed."<< std::endl;
        return nullptr;
    }


    //DESCRIPTOR FILTERING: CROSS CHECK: TODO


    //DESCRIPTOR FILTERING: DISTANCE FILTERING: TODO

    
    //PIXEL MATCHING BETWEEN FRAMES, USE DMATCH RESULTS
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    std::vector<cv::DMatch> validMatches;
    buildValidCorrespondences(points1, points2, validMatches);


    //COUNT LANDMARKS AND DECIDE BETWEEN ESSENTIAL MATRIC VS RANSAC
    //IF E: NO BASELINE CHECK WITH NORMALIZED ARBITRARY SCALE
    std::cout<<"Landmarks count: "<<mapManager_.getLandmarkCount()<<std::endl;// SELECT POSE ESTIMATION METHOD BASED ON MAP LANDMARK COUNT
    const bool usePnP = mapManager_.getLandmarkCount() >= 6;


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

    //RANSAC: E+RECOVER POSE or PNP BASED ON NUMBER OF LANDMARKS
    if (!estimateFramePose(
        points1,
        points2,
        validMatches,
        usePnP,
        essentialMatrix,
        inlierMask,
        R,
        t,
        poseInlierMask,
        pnpResult))
    {
        return nullptr;
    }


    //PRINT FRAME POSE: OPTIONAL (FOR LOG)
    //E+RECOVER POSE GIVES RELATIVE POSE BUT PNP GIVES ABSOLUTE POSE


    //INITIAL TENTATIVE AND ACTIVE TRACKS UPDATE
    //masks should have dimensions predictable for indexing: RESHAPE METHOD
    cv::Mat finalMask = poseInlierMask.reshape(1, 1);
    if (finalMask.type() != CV_8UC1 ||
    finalMask.total() != validMatches.size())
    {
        std::cerr<< "Pose inlier mask does not match matches_."<< std::endl;
        return nullptr;
    }
    std::vector<std::uint8_t> geometricInlierMask =initialTrackUpdate(finalMask,validMatches,currentFrameId);
    //CHALLENGES FACED IN INITIAL TRACK UPDATE
    //1. Too many tentative tracks: 6000-86
    //2. Features points were not being added to triangulated and landmarks


    //MOTION GATING: GEOMETRIC VERIFICATION: PNP AND ESSENTIAL MATRIX
    //VALID FEATURES BASED ON CAMERA MOTION
    //DOES NOT AFFECT FEATURES OR TRACKS
    //ONLY DISCARDS FEATURES IF FAILS
    // MOTION GATING
    if (!evaluateMotionGating(
            validMatches,
            inlierMask,
            finalMask,
            usePnP,
            R,
            t,
            pnpResult,
            currentFrameId))
    {
        return nullptr;
    }


    //PARALLAX AND BASELINE CHECKS FOR TRACKS
    //DOES NOT DISCARD FRAME, KEYFRAME, NO DEMOTION OF TRACKS
    //MARK TRIANGULATION TRACKS
    const std::vector<TrackId> triangulationCandidates =parallaxBaselineCheck(currentFrameId, usePnP);
    std::cout << "Triangulation candidates: "<< triangulationCandidates.size() << std::endl;

    //CHALLENGES FACED IN PARALLEX AND BASELINE
    //1. Baseline not to be used without pnp, arbitrary and normalized scale
    //2. Parallax threshold relaxation brings better results
    //3. Checked low trigulation candidates with ground truth pose comparison: relevant
    //4. Do not check parallax from previous frame but from the last keyframe of current track. TODO
    
    
    
    //POSE ONLY REFINEMENT: MINIMIZE THE REPROJECTION ERROR: TODO
    //AT LEAST 20 CORRESPONCECES BETWEN FEATURES AND LANDMARKS


    //TRIANGULATION
    //PARALLAX CHECK
    //BASELINE CHECK IF PNP
    //TRACK STATE: ACTIVE ONLY, NO TENTATIVE
    //NO LANDMARK SHOULD BE PREVIOUS FORMED FOR TRACK
    //NUMBER OF OBSERVATIONS: 3
    //CHECK IF THE POINT IS ALREADY TRIANGULATED

    MidpointTriangulatorConfig triangulatorConfig;
    triangulatorConfig.minParallaxDegrees = 0.3;
    MidpointTriangulator triangulator(triangulatorConfig);

    Eigen::Matrix3d intrinsicMatrix;
    intrinsicMatrix << cameraCalibration_.fx, 0.0, cameraCalibration_.cx,
                       0.0, cameraCalibration_.fy, cameraCalibration_.cy,
                       0.0, 0.0, 1.0;

    const auto makeWorldToCamera = [](const CameraPose& pose)
    {
        Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
        const Eigen::Matrix3d Rcw = pose.worldFromCameraRotation.transpose();
        Tcw.block<3, 3>(0, 0) = Rcw;
        Tcw.block<3, 1>(0, 3) = -Rcw * pose.positionWorld;
        return Tcw;
    };

    if (!prevFramePose_.valid || !currFramePose_.valid) return nullptr;

    const Eigen::Matrix4d previousTcw = makeWorldToCamera(prevFramePose_);
    const Eigen::Matrix4d currentTcw = makeWorldToCamera(currFramePose_);

    std::unordered_set<TrackId> newlyTriangulatedTracks;
    int triangulatedTracks=0;

    for (const TrackId trackId : triangulationCandidates)
    {
        IFeatureTrack* track = trackManager_->getTrack(trackId);
        if (track == nullptr) continue;
        if (track->getState() == TrackState::Triangulated) continue;

        const FeatureObservation* previousObservation = nullptr;
        const FeatureObservation* currentObservation = nullptr;
        for (const auto& observation : track->getObservations())
        {
            if (observation.cameraId != cameraId_) continue;
            if (observation.frameId == prevFrameId_) previousObservation = &observation;
            else if (observation.frameId == currentFrameId) currentObservation = &observation;
        }

        if (previousObservation == nullptr || currentObservation == nullptr) continue;

        TriangulationObservation first;
        first.pixel = cv::Point2d(previousObservation->pixel.x(), previousObservation->pixel.y());
        first.Tcw = previousTcw;
        first.K = intrinsicMatrix;

        TriangulationObservation second;
        second.pixel = cv::Point2d(currentObservation->pixel.x(), currentObservation->pixel.y());
        second.Tcw = currentTcw;
        second.K = intrinsicMatrix;

        const TriangulationResult result = triangulator.triangulate(first, second);
        if (!result.valid()) continue;

        //VALIDATE THE TRIANGULATION
        //REPROJECTION ERROR IN CURRENT FRAME AND PREV FRAME
        //CHIRALITY CHECK IN PREV FRAME
        //NUMERICAL VALUES SHOULD BE VALID

        const Eigen::Vector2d previousPixel(
            previousObservation->pixel.x(),
            previousObservation->pixel.y());

        const Eigen::Vector2d currentPixel(
            currentObservation->pixel.x(),
            currentObservation->pixel.y());

        if (!validateTriangulatedPoint(
        result.pointWorld,
        previousPixel,
        currentPixel,
        previousTcw,
        currentTcw,
        intrinsicMatrix))
        {
            //std::cout<<"Triangulation init failed for track."<<std::endl;
            continue;
        }

        //PROMOTE THE TRACK IF TRNAGULATION SUCCESS
        // Store the validated 3D point and mark the track as triangulated.
        track->setTriangulatedPointWorld(result.pointWorld);

        if (!trackManager_->markTriangulated(trackId))
        {
            track->clearTriangulatedPointWorld();
            continue;
        }
        newlyTriangulatedTracks.insert(trackId);
        ++triangulatedTracks;


        // std::cout << "[Triangulation] track=" << trackId
        //           << " point_world=" << result.pointWorld.transpose()
        //           << std::endl;
    }
    //CHALLENGES FACED IN TRIANGULATION
    //1. Segmentation fault: core dumped, very large number of landmarks
    //2. The triangulated point values for same feature point varies largely across each iteration.
    //This is because no landmark was present in the map manager.
    //Check that the point is already triangulated and skip

    // KEYFRAME DETERMINATION BASED ON NUMBER OF VERIFIED TRNAGULATIONS
    //PERCENT OF VALIDATIONS TOO
    std::cout<<"Validated triangulation count: "<<triangulatedTracks<<std::endl;
    const double verifiedRatio =triangulationCandidates.empty()
        ? 0.0: static_cast<double>(triangulatedTracks) /static_cast<double>(triangulationCandidates.size());
    std::cout<<"Validated percent: "<<verifiedRatio<<std::endl;
    if (verifiedRatio>=0.3 && currFramePose_.valid && triangulatedTracks>=10)
    {
        std::cout<<"Checking for keyframe validation"<<std::endl;
        auto keyframe = std::make_shared<Keyframe>(
            currentFrameId,
            frame->timestamp,
            currFramePose_
        );

        if (mapManager_.addKeyframe(keyframe))
        {
            std::cout << "[Keyframe] Added frame "<< currentFrameId << std::endl;
        }
        else{
            std::cout<<"Map manager rejected track."<<std::endl;
        }
    }
    else{
        std::cout<<"ratio or pose invalid."<<std::endl;
    }
    std::cout<<"Total active keyframes: "<<mapManager_.getKeyframeCount()<<std::endl;
    //CHALLENGES FACED WITH KEYFRAME DETERMINATION
    //1. All frames are being changed to keyframes
    //2. Do not rely in total candidates but realized landmarks

    // CHECK FOR TRIANGULATED POINTS
    // MIN CONSECUTIVE INLIERS: 3, MIN FAILURES: 1
    for (const TrackId trackId : trackManager_->getTrackIds())
    {
        IFeatureTrack* track = trackManager_->getTrack(trackId);
        if (track == nullptr ||track->getState() != TrackState::Triangulated)
        {
            continue;
        }

        // Find the track's feature in the current frame.
        const FeatureObservation* currentObservation = nullptr;

        for (const auto& observation : track->getObservations())
        {
            if (observation.cameraId == cameraId_ && observation.frameId == currentFrameId)
            {
                currentObservation = &observation;
                break;
            }
        }

        // No current-frame feature: do not validate or update counters.
        if (currentObservation == nullptr) continue;
        if (newlyTriangulatedTracks.count(trackId) > 0) continue;

        // Newly triangulated tracks have no later-frame observation yet.
        // if (track->getLastFrameId() == currentFrameId) continue;

        const auto& pointWorld = track->getTriangulatedPointWorld();

        if (!pointWorld.has_value())
        {
            track->clearTriangulatedPointWorld();
            track->setState(TrackState::Active);
            continue;
        }

        const bool inlier = matchTriangulatedFeatures(
            *pointWorld,
            *currentObservation,
            currentTcw,
            intrinsicMatrix);

        if (inlier)
        {
            track->setConsecutiveInliers(
                track->getConsecutiveInliers() + 1);

            track->setConsecutiveFailures(0);

            if (track->getConsecutiveInliers() >= 3)
            {
                const LandmarkId landmarkId = nextLandmarkId_++;

                auto landmark = std::make_shared<Landmark>(
                    landmarkId,
                    *pointWorld);

                if (mapManager_.addLandmark(landmark))
                {
                    track->setLandmarkId(landmarkId);
                    track->clearTriangulatedPointWorld();
                    track->setState(TrackState::Landmark);

                    std::cout
                        << "[Triangulated -> Landmark] Track="
                        << trackId
                        << " Landmark=" << landmarkId
                        << std::endl;
                }
            }
        }
        else
        {
            track->setConsecutiveFailures(
                track->getConsecutiveFailures() + 1);

            track->setConsecutiveInliers(0);

            if (track->getConsecutiveFailures() >= 1)
            {
                track->clearTriangulatedPointWorld();
                track->setState(TrackState::Active);
            }
        }
    }

    //CHECK FOR LANDMARKS


    //DECIDE LANDMARKS


    //UPDATE PREV FRAME DATA
    prevKeypoints_ = keypoints_;
    prevDescriptors_ = descriptors_.clone();
    prevFrameId_ = currentFrameId;
    if (currFramePose_.valid) prevFramePose_ = currFramePose_;

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

//UTILITY FOR PROCESS VISUAL
bool MonocularVisualFrontend::estimateFramePose(
    const std::vector<cv::Point2f>& points1,
    const std::vector<cv::Point2f>& points2,
    const std::vector<cv::DMatch>& validMatches,
    bool usePnP,
    cv::Mat& essentialMatrix,
    cv::Mat& inlierMask,
    cv::Mat& R,
    cv::Mat& t,
    cv::Mat& poseInlierMask,
    PnPResult& pnpResult)
{
    if (usePnP)
    {
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        std::vector<std::size_t> pnpMatchIndices;

        pnpMatchIndices.reserve(validMatches.size());
        objectPoints.reserve(validMatches.size());
        imagePoints.reserve(validMatches.size());

        const auto trackIds = trackManager_->getTrackIds();

        for (std::size_t matchIndex = 0;
             matchIndex < validMatches.size();
             ++matchIndex)
        {
            const auto& match = validMatches[matchIndex];

            if (match.queryIdx < 0 || match.trainIdx < 0 ||
                match.queryIdx >= static_cast<int>(prevKeypoints_.size()) ||
                match.trainIdx >= static_cast<int>(keypoints_.size()))
            {
                continue;
            }

            IFeatureTrack* matchedTrack = nullptr;

            for (const TrackId trackId : trackIds)
            {
                IFeatureTrack* track = trackManager_->getTrack(trackId);
                if (track == nullptr)
                    continue;

                const auto& observations = track->getObservations();

                for (const auto& observation : observations)
                {
                    if (observation.frameId == prevFrameId_ &&
                        observation.featureId ==
                            static_cast<FeatureId>(match.queryIdx) &&
                        observation.cameraId == cameraId_)
                    {
                        matchedTrack = track;
                        break;
                    }
                }

                if (matchedTrack != nullptr)
                    break;
            }

            if (matchedTrack == nullptr)
                continue;

            const auto landmarkId = matchedTrack->getLandmarkId();
            if (!landmarkId.has_value())
                continue;

            const auto landmark = mapManager_.getLandmark(*landmarkId);
            if (!landmark || !landmark->isValid())
                continue;

            const Eigen::Vector3d& position = landmark->getPosition();

            objectPoints.emplace_back(
                static_cast<float>(position.x()),
                static_cast<float>(position.y()),
                static_cast<float>(position.z()));

            imagePoints.push_back(keypoints_[match.trainIdx].pt);
            pnpMatchIndices.push_back(matchIndex);
        }

        if (!pnpRansac_ ||
            objectPoints.size() < 6 ||
            objectPoints.size() != imagePoints.size() ||
            objectPoints.size() != pnpMatchIndices.size() ||
            !pnpRansac_->estimate(
                objectPoints,
                imagePoints,
                cameraMatrix_,
                distCoeffs_,
                pnpResult) ||
            !pnpResult.success)
        {
            std::cerr << "PnP pose estimation failed." << std::endl;
            return false;
        }

        // PnP estimates the world-to-camera pose:
        // X_camera = R * X_world + t.
        R = pnpResult.rotation;
        t = pnpResult.translation;

        // POSE DETERMINATION
        cv::Mat R_CW;
        R.convertTo(R_CW, CV_64F);

        cv::Mat t_CW;
        t.convertTo(t_CW, CV_64F);

        Eigen::Matrix3d R_world_camera;

        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                R_world_camera(r, c) = R_CW.at<double>(r, c);
            }
        }

        Eigen::Vector3d t_world_camera(
            t_CW.at<double>(0),
            t_CW.at<double>(1),
            t_CW.at<double>(2));

        currFramePose_.worldFromCameraRotation =
            R_world_camera.transpose();

        currFramePose_.positionWorld =
            -currFramePose_.worldFromCameraRotation * t_world_camera;

        currFramePose_.valid = true;

        inlierMask = cv::Mat::zeros(
            1,
            static_cast<int>(validMatches.size()),
            CV_8UC1);

        for (std::size_t i = 0;
             i < pnpResult.inlierMask.size() &&
             i < pnpMatchIndices.size();
             ++i)
        {
            if (pnpResult.inlierMask[i] != 0)
            {
                inlierMask.at<std::uint8_t>(
                    0,
                    static_cast<int>(pnpMatchIndices[i])) = 1;
            }
        }

        poseInlierMask = inlierMask.clone();
    }
    else
    {
        if (!estimateEssentialMatrixRANSAC(
                prevKeypoints_,
                keypoints_,
                validMatches,
                cameraMatrix_,
                essentialMatrix,
                inlierMask))
        {
            std::cerr << "Essential matrix estimation failed."
                      << std::endl;
            return false;
        }

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
            std::cerr << "Relative pose recovery failed."
                      << std::endl;
            return false;
        }

        // DETERMINE THE NEW FRAME POSE
        if (prevFramePose_.valid)
        {
            cv::Mat R_rel, t_rel;
            R.convertTo(R_rel, CV_64F);
            t.convertTo(t_rel, CV_64F);

            Eigen::Matrix3d R_current_previous;

            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 3; ++c)
                {
                    R_current_previous(r, c) =
                        R_rel.at<double>(r, c);
                }
            }

            Eigen::Vector3d t_current_previous(
                t_rel.at<double>(0),
                t_rel.at<double>(1),
                t_rel.at<double>(2));

            const double translationNorm =
                t_current_previous.norm();

            if (translationNorm > 1e-8)
            {
                t_current_previous /= translationNorm;

                currFramePose_.worldFromCameraRotation =
                    prevFramePose_.worldFromCameraRotation *
                    R_current_previous.transpose();

                currFramePose_.positionWorld =
                    prevFramePose_.positionWorld -
                    currFramePose_.worldFromCameraRotation *
                    t_current_previous;

                currFramePose_.valid = true;
            }
            else
            {
                currFramePose_.valid = false;
            }
        }
        else
        {
            currFramePose_.valid = false;
        }
    }

    return true;
}

//PIXEL CORRESPONDECE IN TWO FRAMES
void MonocularVisualFrontend::buildValidCorrespondences(
    std::vector<cv::Point2f>& points1,
    std::vector<cv::Point2f>& points2,
    std::vector<cv::DMatch>& validMatches)
{
    points1.reserve(matches_.size());
    points2.reserve(matches_.size());
    validMatches.reserve(matches_.size());

    for (const auto& match : matches_)
    {
        if (match.queryIdx < 0 || match.trainIdx < 0 ||
            match.queryIdx >= static_cast<int>(prevKeypoints_.size()) ||
            match.trainIdx >= static_cast<int>(keypoints_.size()))
        {
            continue;
        }

        const cv::Point2f& p1 = prevKeypoints_[match.queryIdx].pt;
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
}

//INITIAL TRACK UPDATE
std::vector<std::uint8_t>
MonocularVisualFrontend::initialTrackUpdate(
    const cv::Mat& finalMask,
    const std::vector<cv::DMatch>& validMatches,
    FrameId currentFrameId)
{
    // Convert OpenCV's mask to the format expected by TrackManager.
    std::vector<std::uint8_t> geometricInlierMask(validMatches.size(), 0);

    for (std::size_t i = 0; i < validMatches.size(); ++i)
    {
        geometricInlierMask[i] =finalMask.at<std::uint8_t>(0, static_cast<int>(i)) != 0;
    }

    // Create observations for the current keypoints.
    const std::vector<FeatureObservation> currentObservations =
        createObservations(currentFrameId,cameraId_,keypoints_);

    // Update existing tracks and create tentative tracks for
    // unmatched current-frame features.
    trackManager_->update(
        currentObservations,
        validMatches,
        geometricInlierMask);

    // Print track statistics.
    std::cout
        << "Total tracks: " << trackManager_->getTrackIds().size()
        << "\nTentative tracks: "<< trackManager_->getTrackIds(TrackState::Tentative).size()
        << "\nActive tracks: "<< trackManager_->getTrackIds(TrackState::Active).size()
        << std::endl;

    return geometricInlierMask;
}

// MOTION GATING
bool MonocularVisualFrontend::evaluateMotionGating(
    const std::vector<cv::DMatch>& validMatches,
    const cv::Mat& inlierMask,
    const cv::Mat& finalMask,
    bool usePnP,
    const cv::Mat& R,
    const cv::Mat& t,
    const PnPResult& pnpResult,
    FrameId currentFrameId)
{
    MotionGatingInput gatingInput;

    gatingInput.candidateMatchCount =
        static_cast<int>(validMatches.size());

    cv::Mat ransacMask = inlierMask.reshape(1, 1);

    if (ransacMask.type() != CV_8UC1 ||
        ransacMask.total() != validMatches.size())
    {
        std::cerr << "Invalid RANSAC inlier mask."
                  << std::endl;
        return false;
    }

    gatingInput.ransacInlierCount =
        cv::countNonZero(ransacMask);

    gatingInput.rotation = R;
    gatingInput.translation = t;

    if (usePnP)
    {
        gatingInput.method = MotionEstimationType::PnP;
        gatingInput.meanReprojectionError =
            pnpResult.meanReprojectionError;
        gatingInput.positiveDepthCount =
            pnpResult.positiveDepthCount;
        gatingInput.recoverPoseInlierCount = 0;
    }
    else
    {
        gatingInput.method = MotionEstimationType::EssentialMatrix;
        gatingInput.recoverPoseInlierCount = cv::countNonZero(finalMask);
    }

    const MotionGatingResult gatingResult =
        motionGating_->evaluate(gatingInput);

    if (!gatingResult.accepted)
    {
        std::cerr
            << "[MotionGating] Rejected pose. Reason: "
            << static_cast<int>(gatingResult.failure)
            << " | RANSAC ratio: "
            << gatingResult.ransacInlierRatio
            << " | Cheirality ratio: "
            << gatingResult.cheiralityRatio
            << " | Rotation: "
            << gatingResult.rotationDegrees
            << " degrees"
            << std::endl;

        prevKeypoints_ = keypoints_;
        prevDescriptors_ = descriptors_.clone();
        prevFrameId_ = currentFrameId;

        if (currFramePose_.valid)
        {
            prevFramePose_ = currFramePose_;
        }

        return false;
    }

    std::cerr << "Motion gating passed." << std::endl;

    return true;
}

std::vector<TrackId>
MonocularVisualFrontend::parallaxBaselineCheck(
    FrameId currentFrameId,
    bool usePnP)
{
    std::vector<TrackId> triangulationCandidates;

    if (!prevFramePose_.valid ||
        !currFramePose_.valid ||
        !parallaxChecker_)
    {
        return triangulationCandidates;
    }

    bool sufficientBaseline = true;

    if (usePnP && baselineChecker_)
    {
        double baselineDistance = 0.0;

        sufficientBaseline = baselineChecker_->hasSufficientBaseline(
            prevFramePose_.positionWorld,
            currFramePose_.positionWorld,
            baselineDistance);
    }

    if (!sufficientBaseline)
    {
        return triangulationCandidates;
    }

    const auto toBearing = [this](const Eigen::Vector2d& pixel)
    {
        return Eigen::Vector3d(
            (pixel.x() - cameraCalibration_.cx) / cameraCalibration_.fx,
            (pixel.y() - cameraCalibration_.cy) / cameraCalibration_.fy,
            1.0).normalized();
    };

    for (const TrackId trackId :
         trackManager_->getTrackIds(TrackState::Active))
    {
        IFeatureTrack* track = trackManager_->getTrack(trackId);

        if (track == nullptr)
        {
            continue;
        }

        // Already associated with a landmark: do not triangulate again.
        if (track->getLandmarkId().has_value())
        {
            continue;
        }

        const auto& observations = track->getObservations();

        const FeatureObservation* previousObservation = nullptr;
        const FeatureObservation* currentObservation = nullptr;

        for (const auto& observation : observations)
        {
            if (observation.cameraId != cameraId_)
            {
                continue;
            }

            if (observation.frameId == prevFrameId_)
            {
                previousObservation = &observation;
            }
            else if (observation.frameId == currentFrameId)
            {
                currentObservation = &observation;
            }
        }

        if (previousObservation == nullptr ||
            currentObservation == nullptr)
        {
            continue;
        }

        const Eigen::Vector3d bearingPrevious =
            prevFramePose_.worldFromCameraRotation *
            toBearing(previousObservation->pixel);

        const Eigen::Vector3d bearingCurrent =
            currFramePose_.worldFromCameraRotation *
            toBearing(currentObservation->pixel);

        double parallaxDegrees = 0.0;

        const bool sufficientParallax =
            parallaxChecker_->hasSufficientParallax(
                bearingPrevious,
                bearingCurrent,
                parallaxDegrees);

        // Optional diagnostic:
        // std::cout << "[Parallax] track=" << trackId
        //           << " angle_deg=" << parallaxDegrees
        //           << " sufficient=" << sufficientParallax
        //           << std::endl;

        if (sufficientParallax)
        {
            triangulationCandidates.push_back(trackId);
        }
    }

    return triangulationCandidates;
}

//VALIDATE A TRIANGULATED POINT
bool MonocularVisualFrontend::validateTriangulatedPoint(
    const Eigen::Vector3d& pointWorld,
    const Eigen::Vector2d& previousPixel,
    const Eigen::Vector2d& currentPixel,
    const Eigen::Matrix4d& previousTcw,
    const Eigen::Matrix4d& currentTcw,
    const Eigen::Matrix3d& intrinsicMatrix) const
{
    // 1. Numerical validity and configured 3D bounds.
    if (!isPointValid(pointWorld))
        return false;

    // 2. Cheirality in both observing frames.
    if (!isPointInFrontOfCamera(pointWorld, previousTcw) ||
        !isPointInFrontOfCamera(pointWorld, currentTcw))
    {
        return false;
    }

    // 3. Reprojection error in both observing frames.
    const double previousError = computeReprojectionError(
        pointWorld,
        previousPixel,
        previousTcw,
        intrinsicMatrix);

    const double currentError = computeReprojectionError(
        pointWorld,
        currentPixel,
        currentTcw,
        intrinsicMatrix);

    if (previousError > maxTriangulationReprojectionErrorPixels_ ||
        currentError > maxTriangulationReprojectionErrorPixels_)
    {
        return false;
    }

    return true;
}

//TRIANGULATED TRACK UPDATE
void MonocularVisualFrontend::triangulatedTrackUpdated(
    FrameId currentFrameId)
{
    constexpr int MAX_CONSECUTIVE_FAILURES = 3;
    const auto trackIds =trackManager_->getTrackIds(TrackState::Triangulated);
    for (const TrackId trackId : trackIds)
    {
        IFeatureTrack* track =trackManager_->getTrack(trackId);
        if (track == nullptr) continue;
        const auto landmarkId = track->getLandmarkId();
        if (!landmarkId.has_value()) continue;
        const auto& observations = track->getObservations();

        if (observations.empty())
            continue;

        // Only validate tracks observed in the current frame.
        if (observations.back().frameId != currentFrameId)
            continue;

        // TODO: Retrieve the existing landmark from MapManager.
        // TODO: Project it using the current camera pose.
        // TODO: Compute reprojection error against observations.back().
        // TODO: Update consecutive inlier/failure counters.

        // Once the reprojection result is available:
        //
        // if (reprojectionValid)
        // {
        //     track->setConsecutiveInliers(
        //         track->getConsecutiveInliers() + 1);
        //     track->setConsecutiveFailures(0);
        // }
        // else
        // {
        //     track->setConsecutiveInliers(0);
        //     track->setConsecutiveFailures(
        //         track->getConsecutiveFailures() + 1);
        //
        //     if (track->getConsecutiveFailures() >=
        //         MAX_CONSECUTIVE_FAILURES)
        //     {
        //         // Apply your chosen demotion policy.
        //     }
        // }
    }
}

//COMPARE A FEARURE AGAINST A TRINGULATED TRACK
bool MonocularVisualFrontend::matchTriangulatedFeatures(
    const Eigen::Vector3d& pointWorld,
    const FeatureObservation& observation,
    const Eigen::Matrix4d& Tcw,
    const Eigen::Matrix3d& K) const
{
    constexpr double MAX_REPROJECTION_ERROR_PX = 3.0;

    // 1. Validate input values.
    if (!pointWorld.allFinite() ||
        !Tcw.allFinite() ||
        !K.allFinite())
    {
        return false;
    }

    // 2. Transform the existing triangulated point into camera coordinates.
    const Eigen::Vector3d pointCamera =
        Tcw.block<3, 3>(0, 0) * pointWorld +
        Tcw.block<3, 1>(0, 3);

    if (!pointCamera.allFinite())
    {
        return false;
    }

    // 3. Cheirality check: point must be in front of the camera.
    if (pointCamera.z() <= 0.0)
    {
        return false;
    }

    // 4. Project the 3D point into the current image.
    const Eigen::Vector3d projected = K * pointCamera;

    if (!projected.allFinite() || projected.z() <= 0.0)
    {
        return false;
    }

    const Eigen::Vector2d projectedPixel(
        projected.x() / projected.z(),
        projected.y() / projected.z());

    // 5. Obtain the actual current-frame feature location.
    const Eigen::Vector2d observedPixel(
        observation.pixel.x(),
        observation.pixel.y());

    if (!projectedPixel.allFinite() ||
        !observedPixel.allFinite())
    {
        return false;
    }

    // 6. Check reprojection error.
    const double reprojectionError =
        (projectedPixel - observedPixel).norm();

    return std::isfinite(reprojectionError) &&
           reprojectionError <= MAX_REPROJECTION_ERROR_PX;
}