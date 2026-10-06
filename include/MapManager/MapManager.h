#pragma once

#include "MapManager/CovisibilityGraph/CovisibilityGraph.h"
#include "MapManager/PoseGraph/PoseGraph.h"
#include "MapManager/KeyframeDatabase.h"
#include "MapManager/KeyframesLandmarks/Landmark.h"

#include <memory>
#include <unordered_map>

class MapManager
{
public:
    MapManager()
        : covisibilityGraph_(std::make_unique<CovisibilityGraph>()),
          poseGraph_(std::make_unique<PoseGraph>()),
          keyframeDatabase_(std::make_unique<KeyframeDatabase>())
    {
    }

    ~MapManager() = default;

private:
    std::unique_ptr<CovisibilityGraph> covisibilityGraph_;
    std::unique_ptr<PoseGraph> poseGraph_;
    std::unique_ptr<KeyframeDatabase> keyframeDatabase_;

    // Or directly owned containers for map entities
    std::unordered_map<KeyframeId, std::shared_ptr<Keyframe>> keyframes_;
    std::unordered_map<LandmarkId, std::shared_ptr<Landmark>> landmarks_;
};