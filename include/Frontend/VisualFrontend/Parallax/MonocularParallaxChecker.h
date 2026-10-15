#pragma once

#include "Frontend/VisualFrontend/Parallax/IParallaxChecker.h"

class MonocularParallaxChecker final : public IParallaxChecker
{
public:
    explicit MonocularParallaxChecker(
        double minParallaxDegrees = 1.0);

    bool hasSufficientParallax(
        const Eigen::Vector3d& bearing1,
        const Eigen::Vector3d& bearing2,
        double& parallaxDegrees) const override;

private:
    double minParallaxDegrees_;
};