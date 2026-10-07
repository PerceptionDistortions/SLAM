#pragma once

#include "SLAMSystem/SLAMSystem.h"
#include "Configuration/Configs/SystemConfig.h"
#include "Factory/SLAMSystemDependencies.h"

#include <memory>

class MonocularEKFSLAMSystem : public SLAMSystem
{
public:
    //CONSTRUCTOR
    //FEEDS DATA INTO THE PARENT CLASS
    MonocularEKFSLAMSystem(SLAMSystemDependencies dependencies,
    const std::string& cameraName);
    
    //DESTRUCTOR
    ~MonocularEKFSLAMSystem() override = default;

    bool init() override;
    bool update() override;
    void run() override;
    void shutdown() override;

    StateEstimate getState() const;

private:
    std::string camera_name_;

    //BEFORE FEEDING TO FRONTEND
    FrontendData convertCameraDataToFrontendData(
        const CameraData& cameraData) const;
        
    bool initialized_{false};
    bool estimator_initialized_{false};
    bool running_{false};
};