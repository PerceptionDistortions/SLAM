#pragma once

#include "MapManager/KeyframesLandmarks/Keyframe.h"

#include <cstddef>

class CovisibilityEdge
{
public:
    CovisibilityEdge(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2,
        std::size_t weight)
        : keyframeId1_(keyframeId1),
          keyframeId2_(keyframeId2),
          weight_(weight)
    {
    }

    KeyframeId getKeyframeId1() const
    {
        return keyframeId1_;
    }

    KeyframeId getKeyframeId2() const
    {
        return keyframeId2_;
    }

    std::size_t getWeight() const
    {
        return weight_;
    }

    void setWeight(std::size_t weight)
    {
        weight_ = weight;
    }

private:
    KeyframeId keyframeId1_;
    KeyframeId keyframeId2_;
    std::size_t weight_{0};
};