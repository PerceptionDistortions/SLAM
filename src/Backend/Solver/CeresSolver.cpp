#include "Backend/Solver/CeresSolver.h"
#include "Backend/OptimizationProblem/CeresOptimizationProblem.h"

#include <ceres/ceres.h>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace {
ceres::LinearSolverType parseLinearSolver(const std::string& linearSolver)
{
    std::string value = linearSolver;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });

    if (value == "DENSE_QR") return ceres::DENSE_QR;
    if (value == "DENSE_NORMAL_CHOLESKY") return ceres::DENSE_NORMAL_CHOLESKY;
    if (value == "SPARSE_NORMAL_CHOLESKY") return ceres::SPARSE_NORMAL_CHOLESKY;
    if (value == "DENSE_SCHUR") return ceres::DENSE_SCHUR;
    if (value == "SPARSE_SCHUR") return ceres::SPARSE_SCHUR;
    if (value == "ITERATIVE_SCHUR") return ceres::ITERATIVE_SCHUR;
    if (value == "CGNR") return ceres::CGNR;

    throw std::runtime_error("Unsupported Ceres linear solver: " + linearSolver);
}
}

CeresSolver::CeresSolver(std::unique_ptr<SolverStrategy> strategy,
                         CeresSolverConfig config)
    : strategy_(std::move(strategy)), config_(std::move(config)) {}

bool CeresSolver::solve(OptimizationProblem& problem)
{
    auto* ceresProblem = dynamic_cast<CeresOptimizationProblem*>(&problem);
    if (!ceresProblem || !strategy_) return false;

    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.max_num_iterations = config_.maxNumIterations;
    options.max_solver_time_in_seconds = config_.maxSolverTimeSeconds;
    options.function_tolerance = config_.functionTolerance;
    options.gradient_tolerance = config_.gradientTolerance;
    options.parameter_tolerance = config_.parameterTolerance;
    options.num_threads = config_.numThreads;
    options.linear_solver_type = parseLinearSolver(config_.linearSolver);

    switch (strategy_->getAlgorithm())
    {
        case OptimizationAlgorithm::LevenbergMarquardt:
            options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
            break;

        case OptimizationAlgorithm::Dogleg:
            options.trust_region_strategy_type = ceres::DOGLEG;
            break;

        //DO NOT USE IN CONFIG FOR NOW, ELSE ERRORS WILL BE THERE
        case OptimizationAlgorithm::GaussNewton:
            return false;
    }

    std::string validationError;
    if (!options.IsValid(&validationError)) return false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &ceresProblem->getProblem(), &summary);

    return summary.IsSolutionUsable();
}
