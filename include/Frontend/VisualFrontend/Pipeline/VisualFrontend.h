#pragma once

#include "Sensors/SensorDataClasses/MonocularFrame.h"
#include "Estimation/Measurement/Measurement.h"
#include"Frontend/Frontend.h"

#include <memory>

class VisualFrontend:public Frontend
{
public:
    virtual ~VisualFrontend() = default;

    virtual std::unique_ptr<Measurement> process(
        const MonocularFrame& frame) = 0;
};