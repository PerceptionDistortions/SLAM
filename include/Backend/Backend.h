#pragma once

#include <memory>

#include "Backend/BA/LocalBundleAdjustment.h"
#include "Backend/BA/GlobalBundleAdjustment.h"
#include "MapManager/PoseGraph/PoseGraphOptimization.h"
#include "Configuration/NewConfigs/BackendConfig.h"
#include "Sensors/Calibration/DataStructures/CameraCalibration.h"

struct CeresSolverConfig;

class Backend
{
public:
    Backend(const BackendConfig& config,
        std::unique_ptr<LocalBundleAdjustment> localBA,
        std::unique_ptr<GlobalBundleAdjustment> globalBA,
        std::unique_ptr<PoseGraphOptimization> pgo);
    ~Backend();

    void optimizeLocal(MapManager& map, const CameraCalibration& calibration);
    void optimizeGlobal(MapManager& map, const CameraCalibration& calibration);
    void optimizePoseGraph(MapManager& map);

    OptimizationSolver* getLocalSolver() const { return localSolver_.get(); }
    OptimizationSolver* getGlobalSolver() const { return globalSolver_.get(); }
    OptimizationSolver* getEssentialMatrixSolver() const { return essentialMatrixSolver_.get(); }
    OptimizationSolver* getPoseOnlySolver() const { return poseOnlySolver_.get(); }

private:
    std::unique_ptr<OptimizationSolver> createSolver(
        const std::string& solverType,
        const std::string& algorithm,
        const CeresSolverConfig& solverConfig);

    BackendConfig config_;

    std::unique_ptr<OptimizationSolver> localSolver_;
    std::unique_ptr<OptimizationSolver> globalSolver_;
    std::unique_ptr<OptimizationSolver> essentialMatrixSolver_;
    std::unique_ptr<OptimizationSolver> poseOnlySolver_;

    std::unique_ptr<LocalBundleAdjustment> localBA_;
    std::unique_ptr<GlobalBundleAdjustment> globalBA_;
    std::unique_ptr<PoseGraphOptimization> poseGraphOptimization_;
};
