#pragma once

#include <opencv2/core.hpp>

enum class MotionGatingFailure
{
    None,
    InsufficientMatches,
    InsufficientRansacInliers,
    LowRansacInlierRatio,
    InsufficientPoseInliers,
    LowCheiralityRatio,
    InvalidRotation,
    InvalidTranslation,
    InvalidPose,
    ExcessiveRotation
};