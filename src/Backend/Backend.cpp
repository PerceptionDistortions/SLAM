#include "Backend/Backend.h"

Backend::Backend(std::unique_ptr<LocalBundleAdjustment> localBA, std::unique_ptr<GlobalBundleAdjustment> globalBA, std::unique_ptr<PoseGraphOptimization> pgo)
    : localBA_(std::move(localBA)), globalBA_(std::move(globalBA)), poseGraphOptimization_(std::move(pgo)) {}

Backend::~Backend() = default;

void Backend::optimizeLocal(MapManager& map) { if (localBA_) localBA_->optimize(map); }
void Backend::optimizeGlobal(MapManager& map) { if (globalBA_) globalBA_->optimize(map); }
void Backend::optimizePoseGraph(MapManager& map) { if (poseGraphOptimization_) poseGraphOptimization_->optimize(map); }