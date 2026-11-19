#pragma once

#include <memory>

#include "Backend/BA/LocalBundleAdjustment.h"
#include "Backend/BA/GlobalBundleAdjustment.h"
#include "MapManager/PoseGraph/PoseGraphOptimization.h"

class MapManager;

class Backend
{
public:
    Backend(std::unique_ptr<LocalBundleAdjustment> localBA,
            std::unique_ptr<GlobalBundleAdjustment> globalBA,
            std::unique_ptr<PoseGraphOptimization> pgo);

    ~Backend();

    void optimizeLocal(MapManager& map);
    void optimizeGlobal(MapManager& map);
    void optimizePoseGraph(MapManager& map);

private:
    std::unique_ptr<LocalBundleAdjustment> localBA_;
    std::unique_ptr<GlobalBundleAdjustment> globalBA_;
    std::unique_ptr<PoseGraphOptimization> poseGraphOptimization_;
};