
#pragma once

#include <Eigen/Core>

struct PoseCorrespondence
{
    // Existing landmark in world coordinates.
    Eigen::Vector3d pointWorld = Eigen::Vector3d::Zero();

    // Matched feature location in the current image, in pixels.
    Eigen::Vector2d imagePoint = Eigen::Vector2d::Zero();

    [[nodiscard]] bool isValid() const
    {
        return pointWorld.allFinite() &&
               imagePoint.allFinite();
    }
};
