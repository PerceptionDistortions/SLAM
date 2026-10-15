#include "Frontend/VisualFrontend/Parallax/MonocularParallaxChecker.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

MonocularParallaxChecker::MonocularParallaxChecker(
    double minParallaxDegrees)
    : minParallaxDegrees_(minParallaxDegrees)
{
    if (!std::isfinite(minParallaxDegrees_) ||
        minParallaxDegrees_ < 0.0 ||
        minParallaxDegrees_ > 180.0)
    {
        throw std::invalid_argument(
            "Minimum parallax must be between 0 and 180 degrees.");
    }
}

bool MonocularParallaxChecker::hasSufficientParallax(
    const Eigen::Vector3d& bearing1,
    const Eigen::Vector3d& bearing2,
    double& parallaxDegrees) const
{
    parallaxDegrees = 0.0;

    if (!bearing1.allFinite() || !bearing2.allFinite())
        return false;

    const double norm1 = bearing1.norm();
    const double norm2 = bearing2.norm();

    if (norm1 <= 1e-12 || norm2 <= 1e-12)
        return false;

    const Eigen::Vector3d ray1 = bearing1 / norm1;
    const Eigen::Vector3d ray2 = bearing2 / norm2;

    const double cosine = std::clamp(
        ray1.dot(ray2), -1.0, 1.0);

    parallaxDegrees =
        std::acos(cosine) * 180.0 / std::acos(-1.0);

    return parallaxDegrees >= minParallaxDegrees_;
}