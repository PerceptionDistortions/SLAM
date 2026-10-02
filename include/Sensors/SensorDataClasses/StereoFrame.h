#pragma once

#include "Sensors/SensorDataClasses/CameraData.h"

class StereoFrame
{
public:
    CameraData left;
    CameraData right;
};