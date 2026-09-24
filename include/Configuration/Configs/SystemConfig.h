#pragma once

#include <vector>

#include "Configuration/Enums/SLAMType.h"
#include "Configuration/Enums/VisualizerType.h"

#include "DataSourceConfig.h"
#include "SensorConfig.h"
#include "FrontendConfig.h"
#include "EstimatorConfig.h"
#include "BackendConfig.h"
#include "LoopClosureConfig.h"
#include "MapConfig.h"
#include "OutputConfig.h"
#include "EvaluationConfig.h"
#include "LoggingConfig.h"

struct SystemConfig
{
    // ========================================================
    // SYSTEM
    // ========================================================

    struct System
    {
        SLAMType mode{
            SLAMType::Monocular
        };

        VisualizerType visualizer{
            VisualizerType::None
        };

    } system;


    // ========================================================
    // DATA SOURCES
    // ========================================================

    std::vector<DataSourceConfig> dataSources;


    // ========================================================
    // SENSORS
    // ========================================================

    std::vector<SensorConfig> sensors;


    // ========================================================
    // FRONTEND
    // ========================================================

    FrontendConfig frontend;


    // ========================================================
    // ESTIMATOR
    // ========================================================

    EstimatorConfig estimator;


    // ========================================================
    // BACKEND
    // ========================================================

    BackendConfig backend;


    // ========================================================
    // LOOP CLOSURE
    // ========================================================

    LoopClosureConfig loopClosure;


    // ========================================================
    // MAP
    // ========================================================

    MapConfig map;


    // ========================================================
    // OUTPUT
    // ========================================================

    OutputConfig output;


    // ========================================================
    // EVALUATION
    // ========================================================

    EvaluationConfig evaluation;


    // ========================================================
    // LOGGING
    // ========================================================

    LoggingConfig logging;
};