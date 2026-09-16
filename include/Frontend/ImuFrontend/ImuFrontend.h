#pragma once

#include "Sensors/SensorDataClasses/ImuData.h"
#include "Estimation/Measurement/Measurement.h"

#include <memory>

class ImuFrontend
{
public:
    virtual ~ImuFrontend() = default;

    virtual std::unique_ptr<Measurement> process(
        const ImuData& data) = 0;
};