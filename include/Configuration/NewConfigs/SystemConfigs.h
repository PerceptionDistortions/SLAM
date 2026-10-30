#pragma once

#include<vector>

#include"Configuration/NewConfigs/VisualTrackConfig.h"
#include"Configuration/NewConfigs/SLAMConfigs.h"
#include"Configuration/NewConfigs/VisualFrontendConfigs.h"
#include"Configuration/NewConfigs/LoggerConfig.h"
#include"Configuration/NewConfigs/PnpConfig.h"
#include"Configuration/NewConfigs/TriangulationConfig.h"
#include"Configuration/NewConfigs/SensorsConfig.h"
#include"Configuration/NewConfigs/SensorDataSourceConfig.h"

struct SystemConfigs
{
    SLAMConfigs slam;
    SensorDataSourceConfig sensorDataSource;
    std::vector<SensorsConfig> sensors;

    VisualFrontendConfigs visualFrontend;
    VisualTrackConfig visualTracks;

    PnpConfig pnp;
    TriangulationConfig triangulation;
    LoggerConfig logging;
};
