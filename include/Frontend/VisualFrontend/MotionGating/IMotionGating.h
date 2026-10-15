#pragma once

#include"Frontend/VisualFrontend/MotionGating/MotionGatingResult.h"
#include"Frontend/VisualFrontend/MotionGating/MotionGatingInput.h"

class IMotionGating
{
public:
    virtual ~IMotionGating() = default;

    [[nodiscard]]
    virtual MotionGatingResult evaluate(
        const MotionGatingInput& input) const = 0;
};