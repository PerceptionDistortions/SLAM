#pragma once

#include "Sensors/SensorManager/SensorManager.h"
#include "Frontend/FrontendManager.h"
#include "Estimation/Estimator.h"
#include "Visualization/Visualizer.h"
#include "Backend/Backend.h"
#include "LoopClosure/LoopClosure.h"

#include <memory>

//ABSTRACT CLASS
class SLAMSystem
{
public:
    SLAMSystem() = default;
    virtual ~SLAMSystem() = default;
    //INIT EACH SUB SYSTEM: FRONTEND, BACKEND, VISUALIZER, ETC.
    virtual void init() = 0;

    //RUN ONE SLAM INTERATION
    virtual void update() = 0;

    //SLAM WHILE LOOP CALLED IN MAIN: SLAM RUNS CONTINUOUS
    //USES UPDATE
    virtual void run() = 0;

    virtual void shutdown() = 0;

protected:
    //SENSORS, BUFFERS, CALIBRATORS, SENSOR CONTEXT, DRIVERS
    std::unique_ptr<SensorManager> sensor_manager_;

    //HOLDS ALL TYPES OF PROPROCESSING FRONTENDS: OWNS THEM
    //TAKES THE DATA PACKET FROM BUFFER AND ROUTES TO PROPER FRONTEND
    std::unique_ptr<FrontendManager> frontend_;

    //ONE TYPE OF ESTIMATOR REQUIRED: FILTER OR OPTIMIZER
    std::unique_ptr<Estimator> estimator_;

    //MULTIPLE LAYERS OF BACKEND: LOCAL, GLOBAL, POSE GRAPH
    std::unique_ptr<Backend> backend_;

    //A SINGLE ALGORITHM WITH DIFFERENT STRATEGIES
    std::unique_ptr<LoopClosure> loop_closure_;

    //ONE TYPE OF VISUALIZER: PANGOLIN FOR NOW IN LINUX
    std::unique_ptr<Visualizer> visualizer_;
};