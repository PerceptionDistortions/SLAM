#pragma once

#include"MapManager/MapManager.h"
#include"Sensors/Calibration/DataStructures/CameraCalibration.h"
#include"Backend/Solver/OptimizationSolver.h"

class BundleAdjustment
{
public:
    virtual ~BundleAdjustment() = default;
    virtual bool optimize(MapManager& map, const CameraCalibration& calibration,
        OptimizationSolver& solver) = 0;
};