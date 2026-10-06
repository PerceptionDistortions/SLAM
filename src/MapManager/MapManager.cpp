#include "MapManager/MapManager.h"

MapManager::MapManager()
    : covisibilityGraph_(std::make_unique<CovisibilityGraph>()),
      poseGraph_(std::make_unique<PoseGraph>()),
      keyframeDatabase_(std::make_unique<KeyframeDatabase>())
{
}