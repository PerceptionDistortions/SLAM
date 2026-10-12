#pragma once

#include "MapManager/CovisibilityGraph/CovisibilityGraph.h"
#include "MapManager/PoseGraph/PoseGraph.h"
#include "MapManager/KeyframeDatabase.h"
#include "MapManager/KeyframesLandmarks/Landmark.h"

#include <cstddef>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

class MapManager
{
public:
    MapManager();

    ~MapManager() = default;

    //FUNCTIONS TO BE USED BY VISUAL FRONTEND
    bool addKeyframe(std::shared_ptr<Keyframe> keyframe);
    std::shared_ptr<Keyframe> getKeyframe(KeyframeId id) const;
    bool removeKeyframe(KeyframeId id);

    bool addLandmark(std::shared_ptr<Landmark> landmark);
    std::shared_ptr<Landmark> getLandmark(LandmarkId id) const;
    bool removeLandmark(LandmarkId id);

    // Map statistics
    std::size_t getKeyframeCount() const;
    std::size_t getLandmarkCount() const;

    //POSE GRAPH AND COVISIBILITY GRAPH READ AND WRITE FUNCTIONS
    // Graph access: non-owning references
    CovisibilityGraph& getCovisibilityGraph();
    const CovisibilityGraph& getCovisibilityGraph() const;

    PoseGraph& getPoseGraph();
    const PoseGraph& getPoseGraph() const;

private:
    std::unique_ptr<CovisibilityGraph> covisibilityGraph_;
    std::unique_ptr<PoseGraph> poseGraph_;
    std::unique_ptr<KeyframeDatabase> keyframeDatabase_;

    // Or directly owned containers for map entities
    std::unordered_map<KeyframeId, std::shared_ptr<Keyframe>> keyframes_;
    std::unordered_map<LandmarkId, std::shared_ptr<Landmark>> landmarks_;

    mutable std::shared_mutex mutex_;
};