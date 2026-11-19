#include "Backend/Solver/CeresSolver.h"
#include "Backend/OptimizationProblem/CeresOptimizationProblem.h"

#include <ceres/ceres.h>
#include <utility>

CeresSolver::CeresSolver(std::unique_ptr<SolverStrategy> strategy)
    : strategy_(std::move(strategy)) {}

bool CeresSolver::solve(OptimizationProblem& problem)
{
    auto* ceresProblem = dynamic_cast<CeresOptimizationProblem*>(&problem);
    if (!ceresProblem || !strategy_) return false;

    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;

    switch (strategy_->getAlgorithm())
    {
        case OptimizationAlgorithm::LevenbergMarquardt:
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            break;

        case OptimizationAlgorithm::Dogleg:
            options.trust_region_strategy_type = ceres::DOGLEG;
            break;

        case OptimizationAlgorithm::GaussNewton:
            return false;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &ceresProblem->getProblem(), &summary);

    return summary.IsSolutionUsable();
}