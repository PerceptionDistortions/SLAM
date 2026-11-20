#pragma once

#include "Backend/BA/BundleAdjustment.h"

class GlobalBundleAdjustment : public BundleAdjustment
{
public:
    bool optimize(MapManager& map, const CameraCalibration& calibration,
        OptimizationSolver& solver) override;
};