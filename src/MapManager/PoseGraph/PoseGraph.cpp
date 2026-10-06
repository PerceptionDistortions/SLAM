#include "MapManager/PoseGraph/PoseGraph.h"

#include <stdexcept>
#include <algorithm>

void PoseGraph::addNode(
    KeyframeId keyframeId,
    const Eigen::Isometry3d& pose)
{
    if (nodes_.find(keyframeId) != nodes_.end())
    {
        return;
    }

    nodes_[keyframeId] =
        std::make_shared<PoseGraphNode>(
            keyframeId,
            pose);
}

void PoseGraph::removeNode(
    KeyframeId keyframeId)
{
    nodes_.erase(keyframeId);

    // Remove all edges connected to this node.
    edges_.erase(
        std::remove_if(
            edges_.begin(),
            edges_.end(),
            [keyframeId](const std::shared_ptr<PoseGraphEdge>& edge)
            {
                return edge->getKeyframeId1() == keyframeId ||
                       edge->getKeyframeId2() == keyframeId;
            }),
        edges_.end());
}

void PoseGraph::addEdge(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2,
    const Eigen::Isometry3d& relativePose,
    const Eigen::Matrix<double, 6, 6>& information)
{
    if (keyframeId1 == keyframeId2)
    {
        throw std::invalid_argument(
            "PoseGraph: cannot create self-edge.");
    }

    if (!getNode(keyframeId1) ||
        !getNode(keyframeId2))
    {
        throw std::runtime_error(
            "PoseGraph: both nodes must exist before adding an edge.");
    }

    edges_.push_back(
        std::make_shared<PoseGraphEdge>(
            keyframeId1,
            keyframeId2,
            relativePose,
            information));
}

void PoseGraph::removeEdge(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2)
{
    edges_.erase(
        std::remove_if(
            edges_.begin(),
            edges_.end(),
            [keyframeId1, keyframeId2]
            (const std::shared_ptr<PoseGraphEdge>& edge)
            {
                return
                    (edge->getKeyframeId1() == keyframeId1 &&
                     edge->getKeyframeId2() == keyframeId2) ||

                    (edge->getKeyframeId1() == keyframeId2 &&
                     edge->getKeyframeId2() == keyframeId1);
            }),
        edges_.end());
}

std::shared_ptr<PoseGraphNode>
PoseGraph::getNode(
    KeyframeId keyframeId) const
{
    auto it = nodes_.find(keyframeId);

    if (it == nodes_.end())
    {
        return nullptr;
    }

    return it->second;
}

std::shared_ptr<PoseGraphEdge>
PoseGraph::getEdge(
    KeyframeId keyframeId1,
    KeyframeId keyframeId2) const
{
    for (const auto& edge : edges_)
    {
        if ((edge->getKeyframeId1() == keyframeId1 &&
             edge->getKeyframeId2() == keyframeId2) ||

            (edge->getKeyframeId1() == keyframeId2 &&
             edge->getKeyframeId2() == keyframeId1))
        {
            return edge;
        }
    }

    return nullptr;
}

std::vector<KeyframeId>
PoseGraph::getNodeIds() const
{
    std::vector<KeyframeId> ids;

    ids.reserve(nodes_.size());

    for (const auto& [keyframeId, node] : nodes_)
    {
        ids.push_back(keyframeId);
    }

    return ids;
}