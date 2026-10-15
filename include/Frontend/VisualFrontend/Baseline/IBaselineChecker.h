#pragma once

#include <Eigen/Core>

class IBaselineChecker
{
public:
    virtual ~IBaselineChecker() = default;

    virtual bool hasSufficientBaseline(
        const Eigen::Vector3d& cameraPosition1,
        const Eigen::Vector3d& cameraPosition2,
        double& baselineDistance) const = 0;
};