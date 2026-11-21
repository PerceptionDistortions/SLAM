#include "Backend/Backend.h"

#include "Backend/Solver/CeresSolver.h"
#include "Backend/SolverStrategy/LevenbergMarquardtStrategy.h"
#include "Backend/SolverStrategy/DoglegStrategy.h"
#include "Backend/SolverStrategy/GaussNewtonStrategy.h"

#include <stdexcept>
#include <utility>

Backend::Backend(const BackendConfig& config,
                 std::unique_ptr<LocalBundleAdjustment> localBA,
                 std::unique_ptr<GlobalBundleAdjustment> globalBA,
                 std::unique_ptr<PoseGraphOptimization> pgo)
    : config_(config),
      localBA_(std::move(localBA)),
      globalBA_(std::move(globalBA)),
      poseGraphOptimization_(std::move(pgo))
{
    const auto makeCeresConfig = [](int maxNumIterations,
                                    double maxSolverTimeSeconds,
                                    double functionTolerance,
                                    double gradientTolerance,
                                    double parameterTolerance,
                                    int numThreads,
                                    const std::string& linearSolver) {
        return CeresSolverConfig{
            maxNumIterations,
            maxSolverTimeSeconds,
            functionTolerance,
            gradientTolerance,
            parameterTolerance,
            numThreads,
            linearSolver
        };
    };

    localSolver_ = createSolver(
        config_.local_bundle_adjustment_solver,
        config_.local_bundle_adjustment_solver_strategy,
        makeCeresConfig(
            config_.local_ba_ceres_max_num_iterations,
            config_.local_ba_ceres_max_solver_time_seconds,
            config_.local_ba_ceres_function_tolerance,
            config_.local_ba_ceres_gradient_tolerance,
            config_.local_ba_ceres_parameter_tolerance,
            config_.local_ba_ceres_num_threads,
            config_.local_ba_ceres_linear_solver));

    globalSolver_ = createSolver(
        config_.global_bundle_adjustment_solver,
        config_.global_bundle_adjustment_solver_strategy,
        makeCeresConfig(
            config_.global_ba_ceres_max_num_iterations,
            config_.global_ba_ceres_max_solver_time_seconds,
            config_.global_ba_ceres_function_tolerance,
            config_.global_ba_ceres_gradient_tolerance,
            config_.global_ba_ceres_parameter_tolerance,
            config_.global_ba_ceres_num_threads,
            config_.global_ba_ceres_linear_solver));

    essentialMatrixSolver_ = createSolver(
        config_.essential_matrix_solver,
        config_.essential_matrix_solver_strategy,
        makeCeresConfig(
            config_.monocular_em_ceres_max_num_iterations,
            config_.monocular_em_ceres_max_solver_time_seconds,
            config_.monocular_em_ceres_function_tolerance,
            config_.monocular_em_ceres_gradient_tolerance,
            config_.monocular_em_ceres_parameter_tolerance,
            config_.monocular_em_ceres_num_threads,
            config_.monocular_em_ceres_linear_solver));

    poseOnlySolver_ = createSolver(
        config_.visual_frontend_pose_only_solver,
        config_.visual_frontend_pose_only_solver_strategy,
        makeCeresConfig(
            config_.monocular_pose_only_ceres_max_num_iterations,
            config_.monocular_pose_only_ceres_max_solver_time_seconds,
            config_.monocular_pose_only_ceres_function_tolerance,
            config_.monocular_pose_only_ceres_gradient_tolerance,
            config_.monocular_pose_only_ceres_parameter_tolerance,
            config_.monocular_pose_only_ceres_num_threads,
            config_.monocular_pose_only_ceres_linear_solver));
}

Backend::~Backend() = default;

std::unique_ptr<OptimizationSolver> Backend::createSolver(
    const std::string& solverType,
    const std::string& algorithm,
    const CeresSolverConfig& solverConfig)
{
    if (solverType == "ceres")
    {
        std::unique_ptr<SolverStrategy> strategy;

        if (algorithm == "levenberg_marquardt")
            strategy = std::make_unique<LevenbergMarquardtStrategy>();
        else if (algorithm == "dogleg")
            strategy = std::make_unique<DoglegStrategy>();
        else if (algorithm == "gauss_newton")
            strategy = std::make_unique<GaussNewtonStrategy>();
        else
            throw std::runtime_error("Unsupported Ceres solver strategy: " + algorithm);

        return std::make_unique<CeresSolver>(std::move(strategy), solverConfig);
    }

    throw std::runtime_error("Unsupported optimization solver: " + solverType);
}

void Backend::optimizeLocal(MapManager& map, const CameraCalibration& calibration)
{
    if (localBA_ && localSolver_)
        localBA_->optimize(map, calibration, *localSolver_);
}

void Backend::optimizeGlobal(MapManager& map, const CameraCalibration& calibration)
{
    if (globalBA_ && globalSolver_)
        globalBA_->optimize(map, calibration, *globalSolver_);
}

void Backend::optimizePoseGraph(MapManager&)
{
    // Pose-graph optimization has no implementation yet.
}
