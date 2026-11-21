#pragma once

#include "Sensors/SensorDataClasses/CameraData.h"

class MonocularFrame
{
public:
    CameraData camera;
    // Seconds, matching estimator and visual measurement timestamps.
    double timestamp{0.0};
};
