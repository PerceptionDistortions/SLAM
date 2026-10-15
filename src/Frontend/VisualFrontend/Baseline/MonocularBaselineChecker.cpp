#include "Frontend/VisualFrontend/Baseline/MonocularBaselineChecker.h"

#include <cmath>
#include <stdexcept>

MonocularBaselineChecker::MonocularBaselineChecker(
    double minBaselineDistance)
    : minBaselineDistance_(minBaselineDistance)
{
    if (!std::isfinite(minBaselineDistance_) ||
        minBaselineDistance_ < 0.0)
    {
        throw std::invalid_argument(
            "Minimum baseline distance must be finite and non-negative.");
    }
}

bool MonocularBaselineChecker::hasSufficientBaseline(
    const Eigen::Vector3d& cameraPosition1,
    const Eigen::Vector3d& cameraPosition2,
    double& baselineDistance) const
{
    baselineDistance = 0.0;

    if (!cameraPosition1.allFinite() ||
        !cameraPosition2.allFinite())
    {
        return false;
    }

    baselineDistance =
        (cameraPosition2 - cameraPosition1).norm();

    return baselineDistance >= minBaselineDistance_;
}