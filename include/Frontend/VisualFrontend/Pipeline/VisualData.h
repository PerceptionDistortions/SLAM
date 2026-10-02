#pragma once

#include <variant>

#include "Sensors/SensorDataClasses/MonocularFrame.h"
#include "Sensors/SensorDataClasses/StereoFrame.h"
// #include "Sensors/SensorDataClasses/MultiCameraFrame.h"

using VisualData = std::variant<
    MonocularFrame,
    StereoFrame
    // MultiCameraFrame
>;