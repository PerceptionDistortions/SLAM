#pragma once

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include"Configuration/Configs/SystemConfig.h"
#include"Configuration/Configs/FrontendConfig.h"

class MonocularVisualFrontend : public VisualFrontend
{
public:
    //CONSTRUCTOR NEEDS THE SYSTEM CONFIG FRONTEND FOR ALGORITHMS
    //PROVIDED BY FACTORY
    explicit MonocularVisualFrontend(const FrontendConfig& config);

    //DESTRUCTOR
    ~MonocularVisualFrontend() override = default;

    std::unique_ptr<Measurement> process(const MonocularFrame& frame) override;

private:
    //OWN FRONTEND CONFIGURATION
    //CONST BECAUSE SYSTEM CONFIG ALREADY OWNS IT
    const FrontendConfig& config_;

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