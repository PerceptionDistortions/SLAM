#pragma once

#include <memory>
#include <string>

#include "Sensors/Drivers/SensorDriver.h"
#include "Sensors/Calibration/DataStructures/ICalibration.h"

class SensorContext
{
public:

    // CONSTRUCTOR: NAME AND DRIVER
    SensorContext(
        const std::string& name,
        std::unique_ptr<SensorDriver> driver
    );

    bool init();
    bool start();
    bool stop();

    const std::string& getName() const;

    SensorDriver& getDriver();

    // CALIBRATION
    void setCalibration(std::unique_ptr<ICalibration> calibration);

    const ICalibration* getCalibration() const;

private:

    std::string name_;

    // DRIVER THAT PROVIDES DATA FOR THIS SENSOR
    std::unique_ptr<SensorDriver> driver_;

    // CALIBRATION BELONGING TO THIS SENSOR
    std::unique_ptr<ICalibration> calibration_;
};