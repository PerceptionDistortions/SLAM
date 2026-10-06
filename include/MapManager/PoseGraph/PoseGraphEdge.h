#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "MapManager/KeyframesLandmarks/Keyframe.h"

class PoseGraphEdge
{
public:
    PoseGraphEdge(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2,
        const Eigen::Isometry3d& relativePose,
        const Eigen::Matrix<double, 6, 6>& information)
        : keyframeId1_(keyframeId1),
          keyframeId2_(keyframeId2),
          relativePose_(relativePose),
          information_(information)
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

    const Eigen::Isometry3d& getRelativePose() const
    {
        return relativePose_;
    }

    const Eigen::Matrix<double, 6, 6>& getInformation() const
    {
        return information_;
    }

private:
    KeyframeId keyframeId1_;
    KeyframeId keyframeId2_;

    // T_12
    Eigen::Isometry3d relativePose_;

    // Measurement confidence
    Eigen::Matrix<double, 6, 6> information_;
};