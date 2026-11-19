#pragma once

#include <memory>

class MapManager;
class OptimizationSolver;
class OptimizationProblem;

class PoseGraphOptimization
{
public:
    explicit PoseGraphOptimization(std::unique_ptr<OptimizationSolver> solver);
    virtual ~PoseGraphOptimization() = default;

    bool optimize(MapManager& mapManager);

private:
    std::unique_ptr<OptimizationSolver> solver_;

    std::unique_ptr<OptimizationProblem> buildProblem(const MapManager& mapManager);
    void applySolution(MapManager& mapManager, const OptimizationProblem& problem);
};