#include"Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"

#include <utility>

MonocularVisualFrontend::MonocularVisualFrontend(const FrontendConfig& config)
    : config_(config)
{}

//TO BE IMPLEMENTED
std::unique_ptr<Measurement>
MonocularVisualFrontend::process(
    const MonocularFrame& frame)
{}

void MonocularVisualFrontend::preprocessImage(
    const MonocularFrame& frame)
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