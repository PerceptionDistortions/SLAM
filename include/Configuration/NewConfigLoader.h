#pragma once
#include <string>

#include"Configuration/NewConfigs/SystemConfigs.h"

class NewConfigLoader{
public:
    static SystemConfigs load(const std::string& filePath);
private:
    // Parse individual configuration sections.
    static SLAMConfigs parseSLAM(const std::string& value);

    static SensorDataSourceConfig parseSensorDataSource(
        const std::string& value);

    static std::vector<SensorsConfig> parseSensors(
        const std::string& value);

    static VisualFrontendConfigs parseVisualFrontend(
        const std::string& value);

    static VisualTrackConfig parseVisualTracks(
        const std::string& value);

    static PnpConfig parsePnp(const std::string& value);

    static TriangulationConfig parseTriangulation(
        const std::string& value);

    static LoggerConfig parseLogging(const std::string& value);

    static FilterEstimatorConfig parseFilterEstimator(
        const std::string& value);
};