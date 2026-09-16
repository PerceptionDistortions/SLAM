#pragma once

#include "Frontend/VisualFrontend/VisualFrontend.h"

class MonocularVisualFrontend : public VisualFrontend
{
public:
    ~MonocularVisualFrontend() override = default;

    std::unique_ptr<Measurement> process(
        const MonocularFrame& frame) override;

private:
    void preprocessImage(
        const MonocularFrame& frame);

    void detectFeatures();

    void trackFeatures();

    void matchFeatures();

    void estimateMotion();

    void validateMotion();

    void triangulate();

    void manageLandmarks();

    void manageKeyframes();
};