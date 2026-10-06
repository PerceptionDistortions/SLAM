#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "MapManager/MapTypes.h"
#include "MapManager/Pose.h"

class PoseGraphNode
{
public:
    PoseGraphNode(
        KeyframeId keyframeId,
        const Pose& pose)
        : keyframeId_(keyframeId),
          pose_(pose)
    {
    }

    KeyframeId getKeyframeId() const
    {
        return keyframeId_;
    }

    const Pose& getPose() const
    {
        return pose_;
    }

    void setPose(const Pose& pose)
    {
        pose_ = pose;
    }

private:
    KeyframeId keyframeId_;
    Pose pose_;
};