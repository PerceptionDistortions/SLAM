#pragma once

#include "SLAMSystem/SLAMSystem.h"
#include"Sensors/SensorManager/SensorManager.h"

class MonocularEKFSLAMSystem : public SLAMSystem
{
public:
    //CONSTRUCTOR
    MonocularEKFSLAMSystem(
        const SystemConfig& config,
        SLAMSystemDependencies dependencies);
    
    //DESTRUCTOR
    ~MonocularEKFSLAMSystem() override = default;

    void init() override;
    void update() override;
    void run() override;
    void shutdown() override;

    StateEstimate getState() const;

private:
    bool initialized_{false};
    bool estimator_initialized_{false};
    bool running_{false};
};