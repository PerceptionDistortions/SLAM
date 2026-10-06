#pragma once

#include "MapManager/PoseGraph/PoseGraphNode.h"
#include "MapManager/PoseGraph/PoseGraphEdge.h"

#include <memory>
#include <unordered_map>
#include <vector>

class PoseGraph
{
public:
    PoseGraph() = default;
    ~PoseGraph() = default;

    void addNode(
        KeyframeId keyframeId,
        const Eigen::Isometry3d& pose);

    void removeNode(
        KeyframeId keyframeId);

    void addEdge(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2,
        const Eigen::Isometry3d& relativePose,
        const Eigen::Matrix<double, 6, 6>& information);

    void removeEdge(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2);

    std::shared_ptr<PoseGraphNode> getNode(
        KeyframeId keyframeId) const;

    std::shared_ptr<PoseGraphEdge> getEdge(
        KeyframeId keyframeId1,
        KeyframeId keyframeId2) const;

    std::vector<KeyframeId> getNodeIds() const;

private:
    std::unordered_map<
        KeyframeId,
        std::shared_ptr<PoseGraphNode>> nodes_;

    std::vector<std::shared_ptr<PoseGraphEdge>> edges_;
};