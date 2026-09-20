#include "Configuration/ConfigLoader.h"

#include <yaml-cpp/yaml.h>


SystemConfig ConfigLoader::load(
    const std::string& filename)
{
    YAML::Node root = YAML::LoadFile(filename);

    SystemConfig config;

    // --------------------------------------------------
    // System
    // --------------------------------------------------

    config.system.mode =
        root["system"]["mode"].as<std::string>();

    config.system.visualizer =
        root["system"]["visualizer"].as<std::string>();


    // --------------------------------------------------
    // Data source
    // --------------------------------------------------

    const auto dataSources =
        root["data_sources"];

    config.dataSource.type =
        dataSources["type"].as<std::string>();

    config.dataSource.dataset.name =
        dataSources["dataset"]["name"]
            .as<std::string>();

    config.dataSource.dataset.path =
        dataSources["dataset"]["path"]
            .as<std::string>();

    config.dataSource.playback.realtime =
        dataSources["playback"]["realtime"]
            .as<bool>();

    config.dataSource.playback.rate =
        dataSources["playback"]["rate"]
            .as<double>();


    // --------------------------------------------------
    // Sensors
    // --------------------------------------------------

    for (const auto& node : root["sensors"])
    {
        SystemConfig::Sensor sensor;

        sensor.id =
            node["id"].as<std::string>();

        sensor.type =
            node["type"].as<std::string>();

        sensor.source =
            node["source"].as<std::string>();

        sensor.driver =
            node["driver"].as<std::string>();

        const auto sensorConfig =
            node["config"];

        sensor.config.stream =
            sensorConfig["stream"]
                .as<std::string>();

        if (sensorConfig["data_path"])
        {
            sensor.config.dataPath =
                sensorConfig["data_path"]
                    .as<std::string>();
        }

        if (sensorConfig["csv"])
        {
            sensor.config.csv =
                sensorConfig["csv"]
                    .as<std::string>();
        }

        const auto calibration =
            sensorConfig["calibration"];

        if (calibration)
        {
            sensor.config.calibration.type =
                calibration["type"]
                    .as<std::string>();

            sensor.config.calibration.name =
                calibration["name"]
                    .as<std::string>();

            sensor.config.calibration.file =
                calibration["file"]
                    .as<std::string>();
        }

        config.sensors.push_back(
            std::move(sensor));
    }


    // --------------------------------------------------
    // Visual frontend
    // --------------------------------------------------

    const auto frontend =
        root["visual_frontend"];

    config.visualFrontend.detectorDescriptor =
        frontend["detector_descriptor"]
            .as<std::string>();

    config.visualFrontend.featureMatching =
        frontend["feature_matching"]
            .as<std::string>();


    return config;
}