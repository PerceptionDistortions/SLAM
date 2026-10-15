#pragma once

#include "Frontend/VisualFrontend/Baseline/IBaselineChecker.h"

class MonocularBaselineChecker final : public IBaselineChecker
{
public:
    explicit MonocularBaselineChecker(
        double minBaselineDistance = 0.01);

    bool hasSufficientBaseline(
        const Eigen::Vector3d& cameraPosition1,
        const Eigen::Vector3d& cameraPosition2,
        double& baselineDistance) const override;

private:
    double minBaselineDistance_;
};