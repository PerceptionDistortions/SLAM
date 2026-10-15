#pragma once

#include <opencv2/core.hpp>
#include"Frontend/VisualFrontend/MotionGating/MotionGatingFailure.h"

struct MotionGatingResult
{
    bool accepted = false;

    MotionGatingFailure failure =MotionGatingFailure::InvalidPose;

    double ransacInlierRatio = 0.0;
    double cheiralityRatio = 0.0;
    double rotationDegrees = 0.0;
};