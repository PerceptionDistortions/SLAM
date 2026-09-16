#pragma once

#include "Sensors/SensorManager/SensorManager.h"
#include "Frontend/Frontend.h"
#include "Estimation/Estimator.h"
#include "Visualization/Visualizer.h"
#include"Backend/Backend.h"
#include"LoopClosure/LoopClosure.h"

#include <memory>

class SLAMSystem
{
public:
    virtual ~SLAMSystem();

    virtual void init() = 0;
    virtual void update() = 0;
    virtual void run() = 0;
    virtual void shutdown() = 0;

protected:
    std::unique_ptr<SensorManager> sensor_manager_;
    std::unique_ptr<Frontend> frontend_;
    std::unique_ptr<Estimator> estimator_;
    std::unique_ptr<Visualizer> visualizer_;

    std::unique_ptr<Backend> backend_;
    std::unique_ptr<LoopClosure> loop_closure_;
};