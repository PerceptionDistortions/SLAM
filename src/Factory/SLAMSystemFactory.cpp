#include "Factory/SLAMSystemFactory.h"

#include <stdexcept>
#include <utility>

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
SLAMSystemDependencies SLAMSystemFactory::createDependencies(const SystemConfig& config)
{
    SLAMSystemDependencies dependencies;

    //SENSOR MANAGER, BUFFER, CALIBRATION
    dependencies.sensorManager =std::make_unique<SensorManager>();

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
    //CREATES SENSORS
    //CREATES MAPS
    SLAMSystemDependencies dependencies = createDependencies(config);

    //CREATE FRONTEND: NEED THREE TYPES OF FRONTEND
    //FRONTEND CONFIG
    //FRONTEND NEEDS THREE TYPES OF FRONTEND, WE NEED ONE ONLY
    auto visualFrontend =std::make_unique<MonocularVisualFrontend>(config.frontend);
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
    return std::make_unique<MonocularEKFSLAMSystem>(config,std::move(dependencies));
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