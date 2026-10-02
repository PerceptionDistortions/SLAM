#pragma once

#include "Sensors/SensorDataClasses/ImuData.h"
#include "Estimation/Measurement/Measurement.h"
#include "Frontend/Frontend.h"

#include <memory>

class ImuFrontend:public Frontend
{
public:
    virtual ~ImuFrontend() = default;

    virtual std::unique_ptr<Measurement> processImu(
        const ImuData& data) = 0;

    //INIT AND SHUTDOWN ALREADY DEFINED
};