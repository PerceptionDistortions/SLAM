//COMPONENTS COMMON IN ALL SLAM SYSTEMS

#pragma once

#include <memory>

#include"Sensors/Buffers/BufferManager.h"
#include"Frontend/FrontendManager.h"
#include"MapManager/MapManager.h"
#include"Sensors/Calibration/Manager/CalibrationManager.h"
#include"Sensors/SensorManager/SensorManager.h"
#include"Estimation/Estimator.h"
#include"Backend/Backend.h"
#include"LoopClosure/LoopClosure.h"
#include"Visualization/Visualizer.h"

struct SLAMSystemDependencies
{
    std::shared_ptr<BufferManager> bufferManager;
    std::shared_ptr<CalibrationManager> calibrationManager;
    std::unique_ptr<SensorManager> sensorManager;

    std::shared_ptr<MapManager> mapManager;

    std::unique_ptr<FrontendManager> frontend;
    std::unique_ptr<Estimator> estimator;
    std::unique_ptr<Backend> backend;
    std::unique_ptr<LoopClosure> loopClosure;
    std::unique_ptr<Visualizer> visualizer;
};