#pragma once

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include"Configuration/Configs/SystemConfig.h"
#include"Configuration/Configs/FrontendConfig.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"


class MonocularVisualFrontend : public VisualFrontend
{
public:
    // Constructor receives frontend configuration.
    // Created by the factory.
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

    ~MonocularVisualFrontend() override = default;

    std::unique_ptr<Measurement> processVisual(const VisualData& data) override;

    bool init() override;
    void shutdown() override;

private:
    // Frontend configuration owned by SystemConfig.
    const FrontendConfig& config_;

    CameraCalibration cameraCalibration_;

    void preprocessImage(const MonocularFrame& frame);

    void detectFeatures();

    void trackFeatures();

    void matchFeatures();

    void estimateMotion();

    void validateMotion();

    void triangulate();

    void manageLandmarks();

    void manageKeyframes();
};