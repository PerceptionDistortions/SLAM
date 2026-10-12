#include "Factory/VisualFrontendFactory.h"

#include "Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"
#include "Frontend/VisualFrontend/FeatureDetectionDescription/ORBDetectorDescriptor.h"
#include "Frontend/VisualFrontend/FeatureMatchingTracking/BruteForceMatcher.h"
#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include "Frontend/VisualFrontend/CrossCheck/MutualBestCrossCheck.h"
#include "Frontend/VisualFrontend/DistanceDescriptorFilter/FixedDistanceFilter.h"
#include"Configuration/Configs/VisualFrontendConfig.h"
#include"Configuration/NewConfigs/VisualFrontendConfiguration.h"

#include <memory>

std::unique_ptr<VisualFrontend>
VisualFrontendFactory::createMonocularVisualFrontend(
    const VisualFrontendConfiguration& config,
    const CameraCalibration& calibration,
    MapManager& mapManager)
{
    // std::unique_ptr<IFeatureDetectorDescriptor> featureDetectorDescriptor;

    // if (visualConfig.detectorDescriptor.type == "orb")
    // {
    //     featureDetectorDescriptor =std::make_unique<ORBDetectorDescriptor>(
    //             visualConfig.detectorDescriptor.nFeatures,
    //             visualConfig.detectorDescriptor.scaleFactor,
    //             visualConfig.detectorDescriptor.nLevels);
    // }
    // else
    // {
    //     throw std::runtime_error("Unsupported feature detector/descriptor: " +
    //         visualConfig.detectorDescriptor.type);
    // }

    // //FEATURE MATCHER
    // //BASED ON CONFIG
    // //NORMS: HAMMING, ETC.
    // std::unique_ptr<IFeatureMatcher> featureMatcher;

    // if (visualConfig.featureMatching.type == "bf")
    // {
    //     int normType;
    //     if (visualConfig.featureMatching.descriptor == "hamming")
    //     {
    //         normType = cv::NORM_HAMMING;
    //     }
    //     else
    //     {
    //         throw std::runtime_error(
    //             "Unsupported descriptor distance: " +
    //             visualConfig.featureMatching.descriptor);
    //     }
    //     featureMatcher =std::make_unique<BruteForceMatcher>(normType);
    // }
    // else
    // {
    //     throw std::runtime_error(
    //         "Unsupported feature matcher: " +
    //         visualConfig.featureMatching.type);
    // }

    // //MOTION BLUR CHECKER
    // // MOTION BLUR CHECKER
    // std::unique_ptr<IMotionBlurChecker> motionBlurChecker;

    // const auto& motionBlurConfig =
    //     visualConfig.preprocessing.motionBlur;

    // if (motionBlurConfig.enabled)
    // {
    //     if (motionBlurConfig.method == "laplacian")
    //     {
    //         motionBlurChecker =
    //             std::make_unique<LaplacianBlurChecker>(
    //                 motionBlurConfig.laplacianVariance.threshold
    //             );
    //     }
    //     else
    //     {
    //         throw std::runtime_error(
    //             "Unsupported motion blur method: " +
    //             motionBlurConfig.method
    //         );
    //     }
    // }
    // //LIGHT EXPOSURE CHECKER
    // std::unique_ptr<IExposureCorrector> exposureCorrector;

    // const auto& illuminationConfig =
    //     visualConfig.preprocessing.illumination;

    // if (illuminationConfig.enabled)
    // {
    //     if (illuminationConfig.method == "clahe")
    //     {
    //         exposureCorrector =
    //             std::make_unique<CLAHEExposureCorrector>(
    //                 illuminationConfig.clahe.clipLimit,
    //                 illuminationConfig.clahe.tileGridSize
    //             );
    //     }
    //     else
    //     {
    //         throw std::runtime_error(
    //             "Unsupported illumination method: " +
    //             illuminationConfig.method
    //         );
    //     }
    // }

    // //CROSS CHECKER
    // std::unique_ptr<ICrossCheckStrategy> crossChecker;

    // const auto& crossCheckConfig =visualConfig.crossCheck;

    // if (visualConfig.featureMatching.crossCheck)
    // {
    //     if (crossCheckConfig.type == "mutual_best")
    //     {
    //         crossChecker =std::make_unique<MutualBestCrossCheck>();
    //     }
    //     else
    //     {
    //         throw std::runtime_error(
    //             "Unsupported cross-check strategy: " +
    //             crossCheckConfig.type
    //         );
    //     }
    // }

    // //DISTANCE DESCRIPTOR CHECKER
    // std::unique_ptr<IDistanceFilter> distanceFilter;
    // const auto& distanceFilterConfig =visualConfig.distanceFiltering;

    // std::cout
    //     << "Distance filtering type = ["
    //     << distanceFilterConfig.type
    //     << "]"
    //     << std::endl;

    // std::cout
    //     << "Distance filtering max distance = "
    //     << distanceFilterConfig.maxDistance
    //     << std::endl;

    // if (distanceFilterConfig.type == "fixed_distance")
    // {
    //     distanceFilter =
    //         std::make_unique<FixedDistanceFilter>(
    //             distanceFilterConfig.maxDistance
    //         );
    // }
    // else
    // {
    //     throw std::runtime_error(
    //         "Unsupported distance filtering strategy: " +
    //         distanceFilterConfig.type
    //     );
    // }
    // //VISUAL FRONTEND
    // return std::make_unique<MonocularVisualFrontend>(
    //     config.frontend,
    //     *cameraCalibration,
    //     std::move(featureDetectorDescriptor),
    //     std::move(featureMatcher),
    //     std::move(motionBlurChecker),
    //     std::move(exposureCorrector),
    //     std::move(crossChecker),
    //     std::move(distanceFilter),
    //     *dependencies.mapManager
    // );

    return nullptr;
}