#pragma once

#include <Eigen/Core>
#include <cstdint>

#include "AliasesEnumsTypes/CommonAliases.h"

struct FeatureObservation
{
    slam::FrameId frameId{0};
    slam::FeatureId featureId{0};
    slam::CameraId cameraId{0}; //for stereo and multi view

    Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
};