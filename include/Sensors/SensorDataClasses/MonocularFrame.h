#pragma once

#include "Sensors/SensorDataClasses/CameraData.h"

class MonocularFrame
{
public:
    CameraData camera;
    int64_t timestamp;
};