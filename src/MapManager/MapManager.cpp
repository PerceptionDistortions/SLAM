#include "MapManager/MapManager.h"

MapManager::MapManager()
    : covisibilityGraph_(std::make_unique<CovisibilityGraph>()),
      poseGraph_(std::make_unique<PoseGraph>()),
      keyframeDatabase_(std::make_unique<KeyframeDatabase>())
{
}

bool MapManager::addKeyframe(std::shared_ptr<Keyframe> keyframe)
{
    if (!keyframe)
        return false;

    const KeyframeId id = keyframe->getId();

    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        if (keyframes_.find(id) != keyframes_.end())
            return false;

        keyframes_.emplace(id, keyframe);
    }

    // Register the keyframe with the covisibility graph.
    covisibilityGraph_->addKeyframe(id);

    // Register it with the keyframe database.
    keyframeDatabase_->addKeyframe(keyframe);

    return true;
}

std::shared_ptr<Keyframe>
MapManager::getKeyframe(KeyframeId id) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);

    const auto it = keyframes_.find(id);

    if (it == keyframes_.end())
        return nullptr;

    return it->second;
}

bool MapManager::removeKeyframe(KeyframeId id)
{
    std::shared_ptr<Keyframe> keyframe;

    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        const auto it = keyframes_.find(id);

        if (it == keyframes_.end())
            return false;

        keyframe = it->second;
        keyframes_.erase(it);
    }

    // Remove covisibility graph connections.
    covisibilityGraph_->removeKeyframe(id);

    // Remove from the keyframe database.
    keyframeDatabase_->removeKeyframe(keyframe);

    // Remove observations associated with this keyframe
    // from the reverse map relationships when implemented.
    //
    // If landmarks maintain observing-keyframe lists, update
    // those lists here as well.

    return true;
}

// ============================================================
// LANDMARK MANAGEMENT
// ============================================================

bool MapManager::addLandmark(std::shared_ptr<Landmark> landmark)
{
    if (!landmark)
        return false;

    const LandmarkId id = landmark->getId();

    std::unique_lock<std::shared_mutex> lock(mutex_);

    if (landmarks_.find(id) != landmarks_.end())
        return false;

    landmarks_.emplace(id, std::move(landmark));

    return true;
}

std::shared_ptr<Landmark>
MapManager::getLandmark(LandmarkId id) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);

    const auto it = landmarks_.find(id);

    if (it == landmarks_.end())
        return nullptr;

    return it->second;
}

bool MapManager::removeLandmark(LandmarkId id)
{
    std::vector<std::shared_ptr<Keyframe>> keyframes;

    {
        std::unique_lock<std::shared_mutex> lock(mutex_);

        const auto landmarkIt = landmarks_.find(id);

        if (landmarkIt == landmarks_.end())
            return false;

        // Snapshot keyframes while the map is protected.
        keyframes.reserve(keyframes_.size());

        for (const auto& [keyframeId, keyframe] : keyframes_)
        {
            if (keyframe)
                keyframes.push_back(keyframe);
        }

        // Remove the landmark from the map.
        landmarks_.erase(landmarkIt);
    }

    // Remove all observations referencing this landmark.
    for (const auto& keyframe : keyframes)
    {
        keyframe->removeObservationByLandmarkId(id);
    }

    return true;
}

// ============================================================
// MAP STATISTICS
// ============================================================

std::size_t MapManager::getKeyframeCount() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return keyframes_.size();
}

std::size_t MapManager::getLandmarkCount() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return landmarks_.size();
}

// ============================================================
// COVISIBILITY GRAPH ACCESS
// ============================================================

CovisibilityGraph& MapManager::getCovisibilityGraph()
{
    return *covisibilityGraph_;
}

const CovisibilityGraph&
MapManager::getCovisibilityGraph() const
{
    return *covisibilityGraph_;
}

// ============================================================
// POSE GRAPH ACCESS
// ============================================================

PoseGraph& MapManager::getPoseGraph()
{
    return *poseGraph_;
}

const PoseGraph& MapManager::getPoseGraph() const
{
    return *poseGraph_;
}