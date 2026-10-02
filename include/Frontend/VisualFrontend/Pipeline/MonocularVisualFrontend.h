#pragma once

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include"Configuration/Configs/SystemConfig.h"
#include"Configuration/Configs/FrontendConfig.h"

class MonocularVisualFrontend : public VisualFrontend
{
public:
    // Constructor receives frontend configuration.
    // Created by the factory.
    explicit MonocularVisualFrontend(const FrontendConfig& config);

    ~MonocularVisualFrontend() override = default;

    std::unique_ptr<Measurement> processVisual(
        const VisualData& data) override;

    bool init() override;
    void shutdown() override;

private:
    // Frontend configuration owned by SystemConfig.
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