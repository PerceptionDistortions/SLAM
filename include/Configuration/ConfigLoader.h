#pragma once

#include <string>

#include "Configuration/Configs/SystemConfig.h"

class ConfigLoader
{
public:

    static SystemConfig load(
        const std::string& filePath);

private:

    static SLAMType parseSLAMType(
        const std::string& value);

    static VisualizerType parseVisualizerType(
        const std::string& value);

    static DataSourceType parseDataSourceType(
        const std::string& value);

    static EstimatorType parseEstimatorType(
        const std::string& value);

    static PredictorType parsePredictorType(
        const std::string& value);

    static BackendOptimizerType parseOptimizerType(
        const std::string& value);
};