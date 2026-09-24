#pragma once

#include <string>

#include "Sensors/Calibration/Config/CalibrationConfig.h"

struct SensorConfig
{
    std::string id;

    std::string type;

    std::string sourceId;

    std::string driver;


    struct Dataset
    {
        std::string stream;
        std::string dataPath;
        std::string csv;

    } dataset;


    struct Physical
    {
        std::string device;
        int baudRate{0};

    } physical;


    CalibrationConfig calibration;
};