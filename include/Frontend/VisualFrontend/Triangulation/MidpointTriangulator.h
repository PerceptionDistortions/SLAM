#pragma once

#include "Frontend/VisualFrontend/Triangulation/ITriangulator.h"

struct MidpointTriangulatorConfig
{
    // Reject rays with less angular separation.
    double minParallaxDegrees = 1.0;

    // Minimum distance between camera centers, in pose units.
    double minBaseline = 1e-6;

    // Maximum allowed reprojection error in pixels, per observation.
    double maxReprojectionErrorPixels = 3.0;

    // Maximum distance between the two closest ray points,
    // expressed in the same units as the camera poses.
    // Zero disables this optional check.
    double maxRaySeparation = 0.0;

    // Minimum positive depth in camera coordinates.
    double minDepth = 1e-6;

    // Numerical tolerance for the ray-intersection denominator.
    double geometryEpsilon = 1e-10;
};

class MidpointTriangulator final : public ITriangulator
{
public:
    explicit MidpointTriangulator(
        MidpointTriangulatorConfig config = {});

    [[nodiscard]]
    TriangulationResult triangulate(
        const TriangulationObservation& first,
        const TriangulationObservation& second) const override;

private:
    MidpointTriangulatorConfig config_;
};