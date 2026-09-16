#pragma once

#include "Sensors/SensorDataClasses/ImuData.h"
#include "Sensors/SensorDataClasses/MonocularFrame.h"
#include "Estimation/Measurement/Measurement.h"

#include <memory>

class ImuFrontend;
class VisualFrontend;
class LidarFrontend;

class Frontend
{
public:
    Frontend(
        std::unique_ptr<ImuFrontend> imuFrontend,
        std::unique_ptr<VisualFrontend> visualFrontend,
        std::unique_ptr<LidarFrontend> lidarFrontend);

    ~Frontend();

    std::unique_ptr<Measurement> processIMU(
        const ImuData& data);

    std::unique_ptr<Measurement> processMonocular(
        const MonocularFrame& frame);

private:
    std::unique_ptr<ImuFrontend> _imuFrontend;
    std::unique_ptr<VisualFrontend> _visualFrontend;
    std::unique_ptr<LidarFrontend> _lidarFrontend;
};