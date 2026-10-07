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
}

//OVERRIDEN METHODS
bool MonocularVisualFrontend::init(){
    return true;
}

std::unique_ptr<Measurement> MonocularVisualFrontend::processVisual(const VisualData& data){
    std::cout<<"Monocular visual frontend frame processing."<<std::endl;
    
    return nullptr;
}

void MonocularVisualFrontend::shutdown(){
    
}


void MonocularVisualFrontend::preprocessImage(const MonocularFrame& frame)
{
    // Image preprocessing
}

void MonocularVisualFrontend::detectFeatures()
{
    // Feature detection
}

void MonocularVisualFrontend::trackFeatures()
{
    // Feature tracking
}

void MonocularVisualFrontend::matchFeatures()
{
    // Feature matching
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