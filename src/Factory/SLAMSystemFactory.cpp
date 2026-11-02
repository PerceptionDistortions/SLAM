#include "Factory/SLAMSystemFactory.h"

#include <stdexcept>
#include <utility>
#include <iostream>
// SLAM SYSTEMS
#include "SLAMSystem/MonocularEKFSLAMSystem.h"
#include "SLAMSystem/MonocularVISLAMSystem.h"
#include "SLAMSystem/StereoVISlamSystem.h"

// SENSOR MANAGEMENT
#include "Sensors/SensorManager/SensorManager.h"

// VISUAL FRONTEND
#include "Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"
#include "Frontend/VisualFrontend/Pipeline/StereoVisualFrontend.h"

// ESTIMATORS
#include "Estimation/Filter/ESEKF/ESEKF.h"
#include "Estimation/Optimization/OptimizationEstimator.h"

// FILTER: PREDICTOR AND CORRECTOR
#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"
#include "Estimation/Filter/Corrector/MonocularVisualCorrector.h"
#include "Estimation/Filter/Corrector/StereoVisualCorrector.h"

//OPTIMIZER ESTIMATION

// BACKEND
#include "Backend/Backend.h"

// Loop closure
#include "LoopClosure/LoopClosure.h"

// Map
#include "MapManager/MapManager.h"

// Visualization
#include "Visualization/PangolinVisualizer.h"

//CALLED AND OWNED BY MAIN APP
#include "Factory/SLAMSystemFactory.h"

#include <stdexcept>
#include <utility>

#include "Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"
#include"Frontend/VisualFrontend/FeatureDetectionDescription/ORBDetectorDescriptor.h"
#include"Frontend/VisualFrontend/FeatureMatchingTracking/BruteForceMatcher.h"
#include"Frontend/VisualFrontend/CrossCheck/MutualBestCrossCheck.h"
#include"Frontend/VisualFrontend/DistanceDescriptorFilter/FixedDistanceFilter.h"
#include "Frontend/VisualFrontend/MotionBlurCheck/IMotionBlurChecker.h"
#include "Frontend/VisualFrontend/MotionBlurCheck/LaplacianBlurChecker.h"

#include "Frontend/VisualFrontend/LightExposureCheck/IExposureCorrector.h"
#include "Frontend/VisualFrontend/LightExposureCheck/CLAHEExposureCorrector.h"


#include "Estimation/Filter/ESEKF/ESEKF.h"
#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"
#include "Estimation/Filter/Corrector/MonocularVisualCorrector.h"

#include "SLAMSystem/MonocularEKFSLAMSystem.h"

#include "Backend/Backend.h"
#include "LoopClosure/LoopClosure.h"
#include "Visualization/PangolinVisualizer.h"

#include "Sensors/Calibration/DataStructures/CameraCalibration.h"
#include"Frontend/VisualFrontend/TrackManager/MonocularTrackManager.h"
#include"Frontend/VisualFrontend/MotionGating/MonocularMotionGating.h"
#include"Frontend/VisualFrontend/PnP/IPnPAlgorithm.h"
#include"Frontend/VisualFrontend/PnP/OpenCVEPnPRansac.h"


//TRIGEGRS FACTORY FOR EACH TYPE OF SLAM
//CURRENTLY ONLY 
std::unique_ptr<SLAMSystem> SLAMSystemFactory::create(const SystemConfigs& config)
{
    const std::string& slamType = config.slam.type;

    if (slamType == "mono" || slamType == "monocular")
    {
        return createMonocular(config);
    }

    if (slamType == "mono_vi" || slamType == "monocular_vi")
    {
        throw std::runtime_error("Monocular VI-SLAM not implemented.");
    }

    if (slamType == "stereo_vi")
    {
        throw std::runtime_error("Stereo VI-SLAM not implemented.");
    }

    if (slamType == "multiview_vi")
    {
        throw std::runtime_error(
            "Multiview VI-SLAM not implemented.");
    }

    throw std::runtime_error(
        "Unsupported SLAM type: " + slamType);
}


//ALL NECESSARY MODULES INIT
//SOME COMMON WILL BE CREATED HERE ITSELF
SLAMSystemDependencies SLAMSystemFactory::createDependencies(const SystemConfigs& config,
const std::string& configPath)
{
    SLAMSystemDependencies dependencies;

    //SENSOR MANAGER, BUFFER, CALIBRATION
    dependencies.sensorManager =std::make_unique<SensorManager>();
    if (!dependencies.sensorManager->loadConfig(configPath))
    {
        throw std::runtime_error("Failed to load SensorManager configuration.");
    }

    //MAP MANAGER
    dependencies.mapManager =std::make_unique<MapManager>();

    // OTHER COMMON DEPENDENCIES IF ANY

    return dependencies;
}


std::unique_ptr<SLAMSystem> SLAMSystemFactory::createMonocular(const SystemConfigs& config)
{
    // COMMON DEPENDENCIES: MAP AND SENSORS
    SLAMSystemDependencies dependencies =createDependencies(config, "config/system.yaml");

    //CAMERA CALIBRATION: REQUIRED IN THE VISUAL FRONTEND
    const std::string& cameraName = config.slam.monoCamId;
    auto cameraSensor =dependencies.sensorManager->getSensor(cameraName);
    if (!cameraSensor) throw std::runtime_error("Camera sensor not found: " + cameraName);
    const ICalibration* calibration = cameraSensor->getCalibration();
    const CameraCalibration* cameraCalibration = dynamic_cast<const CameraCalibration*>(calibration);
    if (!cameraCalibration) throw std::runtime_error("Sensor '" + cameraName +"' does not have CameraCalibration.");

    //VISUAL FRONTEND CONFIGURATION    
    const auto& visualConfig = config.visualFrontend;

    //TRIANGULATION CONFIGURATION
    const auto& triangulationConfig=config.triangulation;

    //VISUAL TRACK CONFIGURATION
    const auto& visualTrackConfig=config.visualTracks;

    //FEATURE DETECTION AND DESCRIPTION
    std::unique_ptr<IFeatureDetectorDescriptor> featureDetectorDescriptor;
    if (visualConfig.detectionDescription == "orb")
    {
        featureDetectorDescriptor =std::make_unique<ORBDetectorDescriptor>(
                visualConfig.orbFeatureCount,
                visualConfig.orbScaleFactor,
                visualConfig.orbLevels);
    }
    else
    {
        throw std::runtime_error("Unsupported feature detector/descriptor: " +visualConfig.detectionDescription);
    }

    //FEATURE MATCHING: BF
    std::unique_ptr<IFeatureMatcher> featureMatcher;

    if (visualConfig.featureMatching == "bf")
    {
        // ORB uses binary descriptors.
        featureMatcher =std::make_unique<BruteForceMatcher>(cv::NORM_HAMMING);
    }
    else
    {
        throw std::runtime_error("Unsupported feature matcher: " +visualConfig.featureMatching);
    }

    //MOTION BLUE CHECK
    std::unique_ptr<IMotionBlurChecker> motionBlurChecker;

    if (visualConfig.motionBlur == "laplacian")
    {
        motionBlurChecker =std::make_unique<LaplacianBlurChecker>(visualConfig.laplacianVarianceThreshold);
    }
    else
    {
        throw std::runtime_error("Unsupported motion blur method: " +visualConfig.motionBlur);
    }

    //LIGHT EXPOSURE CHECK: ALGORITHM STRATEGY WILL DEPEND ON CONFIG
    std::unique_ptr<IExposureCorrector> exposureCorrector;
    if (visualConfig.lightExposure == "clahe")
    {
        // Uses the CLAHE component's default configuration.
        exposureCorrector =std::make_unique<CLAHEExposureCorrector>(visualConfig.claheCliplimit,
        visualConfig.claheTileLimitSize);
    }
    else
    {
        throw std::runtime_error("Unsupported exposure correction method: " +visualConfig.lightExposure);
    }

    //DESCRIPTOR CROSS CHECKER ALGO SRATEGY
    std::unique_ptr<ICrossCheckStrategy> crossChecker;

    if (visualConfig.crossCheck == "mutual_best")
    {
        crossChecker =std::make_unique<MutualBestCrossCheck>();
    }
    else if (visualConfig.crossCheck != "none")
    {
        throw std::runtime_error("Unsupported cross-check strategy: " +visualConfig.crossCheck);
    }

    //DESCRIPTOR DISTANCE FILTER
    std::unique_ptr<IDistanceFilter> distanceFilter;

    if (visualConfig.distanceFiltering == "fixed_distance")
    {
        // The current VisualFrontendConfigs does not expose
        // maxDistance. Use the filter's default constructor.
        distanceFilter =
        std::make_unique<FixedDistanceFilter>(visualConfig.fixedDistanceFilterMaxDistance);
    }
    else
    {
        throw std::runtime_error(
            "Unsupported distance filtering strategy: " +
            visualConfig.distanceFiltering);
    }

    //TRACK MANAGER: HARD DEFINED NO NEED OF CONFIG
    std::unique_ptr<ITrackManager> trackManager =
        std::make_unique<MonocularTrackManager>(config.visualTracks);

    //MOTION GATING HARD DEFINED
    std::unique_ptr<IMotionGating> motionGating =std::make_unique<MonocularMotionGating>();

    //PNP
    std::unique_ptr<IPnPAlgorithm> pnpRansac;

    if (config.pnp.pnpStrategy == "epnp")
    {
        pnpRansac =std::make_unique<OpenCVEPnPRansac>(config.pnp);
    }
    else
    {
        throw std::runtime_error("Unsupported PnP strategy: " + config.pnp.pnpStrategy);
    }
    
    
    //MONOCULAR VISUAL FRONTEND OBJECT
    auto visualFrontend =
        std::make_unique<MonocularVisualFrontend>(
            visualConfig,
            triangulationConfig,
            visualTrackConfig,
            *cameraCalibration,
            std::move(featureDetectorDescriptor),
            std::move(featureMatcher),
            std::move(motionBlurChecker),
            std::move(exposureCorrector),
            std::move(crossChecker),
            std::move(distanceFilter),
            std::move(trackManager),
            std::move(motionGating),
            std::move(pnpRansac),
            *dependencies.mapManager,
            cameraName);

    // ---------------------------------------------------------
    // FRONTEND MANAGER
    // ---------------------------------------------------------
    dependencies.frontend =
        std::make_unique<FrontendManager>(nullptr,std::move(visualFrontend),nullptr);

    //ESTIMATOR: RIGID PREDICTOR AND CORRECTOR, AND ESEKF
    auto predictor = std::make_unique<ConstantVelocityPredictor>();
    auto corrector =std::make_unique<MonocularVisualCorrector>();

    dependencies.estimator =
        std::make_unique<ESEKF>(
            std::move(predictor),
            std::move(corrector));

    // ---------------------------------------------------------
    // BACKEND
    // ---------------------------------------------------------
    dependencies.backend = nullptr;

    // ---------------------------------------------------------
    // LOOP CLOSURE
    // ---------------------------------------------------------
    dependencies.loopClosure = nullptr;

    // ---------------------------------------------------------
    // VISUALIZER
    // ---------------------------------------------------------
    dependencies.visualizer = nullptr;

    //FINAL SLAM SYSTEM
    //CONTROLLED PLAYBACK OPTION
    const bool controlledPlayback =
    config.slam.keyBasedControlledPlayback &&
    config.sensorDataSource.type == "dataset";

    auto system = std::make_unique<MonocularEKFSLAMSystem>(
        std::move(dependencies),
        config.slam.monoCamId,
        controlledPlayback);

    return system;
}

//MONOCULAR VI SLAM

//STEREO VI SLAM

//MULTI VIEW SLAM