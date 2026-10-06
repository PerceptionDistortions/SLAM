#pragma once

#include "MapManager/CovisibilityGraph/CovisibilityEdge.h"
#include "MapManager/KeyframesLandmarks/Keyframe.h"

#include <memory>
#include <unordered_map>
#include <vector>

class CovisibilityGraph
{
public:
    CovisibilityGraph() = default;
    ~CovisibilityGraph() = default;

    void addKeyframe(KeyframeId keyframeId);

    void removeKeyframe(KeyframeId keyframeId);

    void addConnection(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2,
        std::size_t weight);

    void removeConnection(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2);

    void updateConnection(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2,
        std::size_t weight);

    std::vector<KeyframeId> getConnectedKeyframes(
        KeyframeId keyframeId) const;

    std::shared_ptr<CovisibilityEdge> getEdge(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2) const;

private:
    using EdgeMap =
        std::unordered_map<
            KeyframeId,
            std::shared_ptr<CovisibilityEdge>>;

    std::unordered_map<KeyframeId, EdgeMap> edges_;
};