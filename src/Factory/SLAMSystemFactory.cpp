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


std::unique_ptr<SLAMSystem> SLAMSystemFactory::create(const SystemConfig& config)
{
    switch (config.system.mode)
    {
        case SLAMType::Monocular: return createMonocular(config);
        case SLAMType::MonocularVI: throw std::runtime_error("Monocular VI-SLAM not implemented.");
        case SLAMType::StereoVI:  throw std::runtime_error("Stereo VI-SLAM not implemented.");
        default:
            throw std::runtime_error("Unsupported SLAM type.");
    }
}


//COMMON DEPENDENCIES
SLAMSystemDependencies SLAMSystemFactory::createDependencies(const SystemConfig& config,
const std::string& configPath)
{
    SLAMSystemDependencies dependencies;

    //SENSOR MANAGER, BUFFER, CALIBRATION
    dependencies.sensorManager =std::make_unique<SensorManager>();
    if (!dependencies.sensorManager->loadConfig(configPath))
    {
        throw std::runtime_error(
            "Failed to load SensorManager configuration.");
    }

    //MAP MANAGER
    dependencies.mapManager =std::make_unique<MapManager>();

    // Other common dependencies...

    return dependencies;
}


// ============================================================================
// Monocular SLAM
// ============================================================================

std::unique_ptr<SLAMSystem>
SLAMSystemFactory::createMonocular(const SystemConfig& config)
{
    //COMMON DEPENDENCIES
    SLAMSystemDependencies dependencies = createDependencies(config,"config/system.yaml");

    //VISUAL FRONTEND CONFIG
    const auto& visualConfig =config.frontend.visual; //VISUAL CONFIG
    
    //
    std::unique_ptr<IFeatureDetectorDescriptor> featureDetectorDescriptor;

    if (visualConfig.detectorDescriptor.type == "orb")
    {
        featureDetectorDescriptor =
            std::make_unique<ORBDetectorDescriptor>(
                visualConfig.detectorDescriptor.nFeatures,
                visualConfig.detectorDescriptor.scaleFactor,
                visualConfig.detectorDescriptor.nLevels);
    }
    else
    {
        throw std::runtime_error(
            "Unsupported feature detector/descriptor: " +
            visualConfig.detectorDescriptor.type);
    }

    //FEATURE MATCHER
    //BASED ON CONFIG
    //NORMS: HAMMING, ETC.
    std::unique_ptr<IFeatureMatcher> featureMatcher;

    if (visualConfig.featureMatching.type == "bf")
    {
        int normType;
        if (visualConfig.featureMatching.descriptor == "hamming")
        {
            normType = cv::NORM_HAMMING;
        }
        else
        {
            throw std::runtime_error(
                "Unsupported descriptor distance: " +
                visualConfig.featureMatching.descriptor);
        }
        featureMatcher =std::make_unique<BruteForceMatcher>(normType);
    }
    else
    {
        throw std::runtime_error(
            "Unsupported feature matcher: " +
            visualConfig.featureMatching.type);
    }

    //MOTION BLUR CHECKER
    // MOTION BLUR CHECKER
    std::unique_ptr<IMotionBlurChecker> motionBlurChecker;

    const auto& motionBlurConfig =
        visualConfig.preprocessing.motionBlur;

    if (motionBlurConfig.enabled)
    {
        if (motionBlurConfig.method == "laplacian")
        {
            motionBlurChecker =
                std::make_unique<LaplacianBlurChecker>(
                    motionBlurConfig.laplacianVariance.threshold
                );
        }
        else
        {
            throw std::runtime_error(
                "Unsupported motion blur method: " +
                motionBlurConfig.method
            );
        }
    }
    //LIGHT EXPOSURE CHECKER
    std::unique_ptr<IExposureCorrector> exposureCorrector;

    const auto& illuminationConfig =
        visualConfig.preprocessing.illumination;

    if (illuminationConfig.enabled)
    {
        if (illuminationConfig.method == "clahe")
        {
            exposureCorrector =
                std::make_unique<CLAHEExposureCorrector>(
                    illuminationConfig.clahe.clipLimit,
                    illuminationConfig.clahe.tileGridSize
                );
        }
        else
        {
            throw std::runtime_error(
                "Unsupported illumination method: " +
                illuminationConfig.method
            );
        }
    }

    //CROSS CHECKER
    std::unique_ptr<ICrossCheckStrategy> crossChecker;

    const auto& crossCheckConfig =
        visualConfig.crossCheck;

    if (visualConfig.featureMatching.crossCheck)
    {
        if (crossCheckConfig.type == "mutual_best")
        {
            crossChecker =
                std::make_unique<MutualBestCrossCheck>();
        }
        else
        {
            throw std::runtime_error(
                "Unsupported cross-check strategy: " +
                crossCheckConfig.type
            );
        }
    }

    //DISTANCE DESCRIPTOR CHECKER
    std::unique_ptr<IDistanceFilter> distanceFilter;
    const auto& distanceFilterConfig =visualConfig.distanceFiltering;

    std::cout
        << "Distance filtering type = ["
        << distanceFilterConfig.type
        << "]"
        << std::endl;

    std::cout
        << "Distance filtering max distance = "
        << distanceFilterConfig.maxDistance
        << std::endl;

    if (distanceFilterConfig.type == "fixed_distance")
    {
        distanceFilter =
            std::make_unique<FixedDistanceFilter>(
                distanceFilterConfig.maxDistance
            );
    }
    else
    {
        throw std::runtime_error(
            "Unsupported distance filtering strategy: " +
            distanceFilterConfig.type
        );
    }
    //VISUAL FRONTEND
    auto visualFrontend =
    std::make_unique<MonocularVisualFrontend>(
        config.frontend,
        std::move(featureDetectorDescriptor),
        std::move(featureMatcher),
        std::move(motionBlurChecker),
        std::move(exposureCorrector),
        std::move(crossChecker),
        std::move(distanceFilter)
    );

    //FRONTEND IN DEPENDENCIES: ONLY VISUAL REQUIRED
    dependencies.frontend =std::make_unique<FrontendManager>(
            nullptr,
            std::move(visualFrontend),
            nullptr);

    //ESTIMATOR: ESEKF
    //PREDICTOR: CONSTANT VELOCITY
    //CORRECTOR: MONOCULAR VISUAL CORRECTOR
    auto predictor =std::make_unique<ConstantVelocityPredictor>();
    auto corrector =std::make_unique<MonocularVisualCorrector>();
    dependencies.estimator =std::make_unique<ESEKF>(
            std::move(predictor),
            std::move(corrector));

    //BACKEND IF ENABLED
    // if (config.backend.enabled)
    // {
    //     dependencies.backend =std::make_unique<Backend>(config.backend,
    //         dependencies.mapManager);
    // }
    dependencies.backend=nullptr;

    //LOOP CLOSURE IF ENABLED
    // if (config.loopClosure.enabled)
    // {
    //     dependencies.loopClosure =
    //         std::make_unique<LoopClosure>(config.loopClosure,dependencies.mapManager);
    // }
    dependencies.loopClosure=nullptr;


    //VISUALIZER: ENABLE ONLY IF TRUE IN CONFIG
    // if (config.system.visualizer == VisualizerType::Pangolin)
    // {
    //     dependencies.visualizer =std::make_unique<PangolinVisualizer>(config);
    // }
    dependencies.visualizer=nullptr;

    //SLAM SYSTEM: FINAL
    return std::make_unique<MonocularEKFSLAMSystem>(std::move(dependencies));
}


// ============================================================================
// Monocular VI-SLAM
// ============================================================================

// std::unique_ptr<SLAMSystem>
// SLAMSystemFactory::createMonocularVI(const SystemConfig& config)
// {
//     SLAMSystemDependencies dependencies = createDependencies(config);

//     //FRONTEND
//     dependencies.frontend = std::make_unique<MonocularVIVisualFrontend>(config);

//     // ------------------------------------------------------------------------
//     // Estimator
//     //
//     // Monocular VI uses optimization-based estimation:
//     //
//     //     OptimizationEstimator
//     //          |
//     //          +-- sliding window
//     //          +-- IMU preintegration
//     //          +-- visual factors
//     //          +-- IMU factors
//     // ------------------------------------------------------------------------

//     dependencies.estimator =std::make_unique<OptimizationEstimator>(config);


//     // ------------------------------------------------------------------------
//     // Backend
//     // ------------------------------------------------------------------------

//     if (config.backend.enabled)
//     {
//         dependencies.backend =
//             std::make_unique<Backend>(
//                 config,
//                 dependencies.mapManager);
//     }


//     // ------------------------------------------------------------------------
//     // Loop Closure
//     // ------------------------------------------------------------------------

//     if (config.loopClosure.enabled)
//     {
//         dependencies.loopClosure =
//             std::make_unique<LoopClosure>(
//                 config,
//                 dependencies.mapManager);
//     }


//     // ------------------------------------------------------------------------
//     // Visualizer
//     // ------------------------------------------------------------------------

//     if (config.system.visualizer == VisualizerType::Pangolin)
//     {
//         dependencies.visualizer =
//             std::make_unique<PangolinVisualizer>(config);
//     }


//     // ------------------------------------------------------------------------
//     // Construct concrete SLAM system
//     // ------------------------------------------------------------------------

//     return std::make_unique<MonocularVISLAMSystem>(
//         config,
//         std::move(dependencies));
// }


// // ============================================================================
// // Stereo VI-SLAM
// // ============================================================================

// std::unique_ptr<SLAMSystem>
// SLAMSystemFactory::createStereoVI(const SystemConfig& config)
// {
//     SLAMSystemDependencies dependencies =
//         createDependencies(config);


//     // ------------------------------------------------------------------------
//     // Frontend
//     // ------------------------------------------------------------------------

//     dependencies.frontend =
//         std::make_unique<StereoVIVisualFrontend>(config);


//     // ------------------------------------------------------------------------
//     // Estimator
//     // ------------------------------------------------------------------------

//     dependencies.estimator =
//         std::make_unique<OptimizationEstimator>(config);


//     // ------------------------------------------------------------------------
//     // Backend
//     // ------------------------------------------------------------------------

//     if (config.backend.enabled)
//     {
//         dependencies.backend =
//             std::make_unique<Backend>(
//                 config,
//                 dependencies.mapManager);
//     }


//     // ------------------------------------------------------------------------
//     // Loop Closure
//     // ------------------------------------------------------------------------

//     if (config.loopClosure.enabled)
//     {
//         dependencies.loopClosure =
//             std::make_unique<LoopClosure>(
//                 config,
//                 dependencies.mapManager);
//     }


//     // ------------------------------------------------------------------------
//     // Visualizer
//     // ------------------------------------------------------------------------

//     if (config.system.visualizer == VisualizerType::Pangolin)
//     {
//         dependencies.visualizer =
//             std::make_unique<PangolinVisualizer>(config);
//     }


//     // ------------------------------------------------------------------------
//     // Construct concrete SLAM system
//     // ------------------------------------------------------------------------

//     return std::make_unique<StereoVISLAMSystem>(
//         config,
//         std::move(dependencies));
// }