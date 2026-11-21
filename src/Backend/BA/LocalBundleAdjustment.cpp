#include "Backend/BA/LocalBundleAdjustment.h"

bool LocalBundleAdjustment::optimize(
    MapManager&,
    const CameraCalibration&,
    OptimizationSolver&)
{
    // Build local BA problem and solve.
    return false;
}
