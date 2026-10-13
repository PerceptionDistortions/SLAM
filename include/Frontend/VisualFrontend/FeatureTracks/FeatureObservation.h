#pragma once

#include <Eigen/Core>
#include <cstdint>

#include "AliasesEnumsTypes/CommonAliases.h"

struct FeatureObservation
{
    FrameId frameId{0};
    FeatureId featureId{0};
    std::string cameraId{"cam0"}; //for stereo and multi view

    Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
};