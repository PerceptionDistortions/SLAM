#pragma once

#include <memory>

#include "Configuration/Configs/SystemConfig.h"
#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include "MapManager/MapManager.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"

class VisualFrontendFactory
{
public:
    static std::unique_ptr<VisualFrontend> createMonocularVisualFrontend(
        const VisualFrontendConfiguration& config,
        const CameraCalibration& calibration,
        MapManager& mapManager);
};