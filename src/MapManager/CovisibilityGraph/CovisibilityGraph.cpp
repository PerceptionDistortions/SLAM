#include "MapManager/CovisibilityGraph/CovisibilityGraph.h"

#include <algorithm>
#include <stdexcept>

void CovisibilityGraph::addKeyframe(KeyframeId keyframeId)
{
    edges_.try_emplace(keyframeId);
}

void CovisibilityGraph::removeKeyframe(KeyframeId keyframeId)
{
    // Remove all edges pointing to this keyframe.
    for (auto& [otherKeyframeId, connections] : edges_)
    {
        if (otherKeyframeId == keyframeId)
            continue;

        connections.erase(keyframeId);
    }

    edges_.erase(keyframeId);
}

void CovisibilityGraph::addConnection(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2,
    std::size_t weight)
{
    if (keyframeId1 == keyframeId2)
    {
        throw std::invalid_argument(
            "CovisibilityGraph: cannot connect keyframe to itself.");
    }

    addKeyframe(keyframeId1);
    addKeyframe(keyframeId2);

    auto edge = std::make_shared<CovisibilityEdge>(
        keyframeId1,
        keyframeId2,
        weight);

    // Store the same edge in both directions.
    edges_[keyframeId1][keyframeId2] = edge;
    edges_[keyframeId2][keyframeId1] = edge;
}

void CovisibilityGraph::removeConnection(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2)
{
    auto it1 = edges_.find(keyframeId1);
    if (it1 != edges_.end())
    {
        it1->second.erase(keyframeId2);
    }

    auto it2 = edges_.find(keyframeId2);
    if (it2 != edges_.end())
    {
        it2->second.erase(keyframeId1);
    }
}

void CovisibilityGraph::updateConnection(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2,
    std::size_t weight)
{
    auto edge = getEdge(keyframeId1, keyframeId2);

    if (!edge)
    {
        addConnection(keyframeId1, keyframeId2, weight);
        return;
    }

    edge->setWeight(weight);
}

std::vector<KeyframeId>
CovisibilityGraph::getConnectedKeyframes(
    KeyframeId keyframeId) const
{
    std::vector<KeyframeId> result;

    auto it = edges_.find(keyframeId);

    if (it == edges_.end())
        return result;

    result.reserve(it->second.size());

    for (const auto& [connectedId, edge] : it->second)
    {
        result.push_back(connectedId);
    }

    return result;
}

std::shared_ptr<CovisibilityEdge>
CovisibilityGraph::getEdge(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2) const
{
    auto it1 = edges_.find(keyframeId1);

    if (it1 == edges_.end())
        return nullptr;

    auto it2 = it1->second.find(keyframeId2);

    if (it2 == it1->second.end())
        return nullptr;

    return it2->second;
}