#pragma once

#include <Eigen/Core>

class IParallaxChecker
{
public:
    virtual ~IParallaxChecker() = default;

    virtual bool hasSufficientParallax(
        const Eigen::Vector3d& bearing1,
        const Eigen::Vector3d& bearing2,
        double& parallaxDegrees) const = 0;
};