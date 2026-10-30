#pragma once

#include<string>

#include "Sensors/Calibration/Config/CalibrationConfig.h"

struct SensorsConfig
{
    std::string id;
    std::string type;
    std::string sourceId;
    std::string driver;

    // Dataset-specific settings
    struct Dataset
    {
        std::string stream;
        std::string dataPath;
        std::string csv;
    } dataset;

    // Physical hardware-specific settings
    struct Physical
    {
        std::string device;
        int baudRate{0};
    } physical;

    // Calibration settings
    CalibrationConfig calibration;
};
