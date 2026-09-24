#pragma once

#include<memory>

#include"Configuration/Configs/SystemConfig.h"
#include "SLAMSystem/SLAMSystem.h"
#include "SLAMSystemDependencies.h"

class SLAMSystemFactory{
public:
    //MAIN CALLS THIS FUNCTION AND THE RETURNED OBJECT IS OWNED BY MAIN: SLAM SYSTEM
    static std::unique_ptr<SLAMSystem> create(const SystemConfig& config);
    
private:
    static SLAMSystemDependencies createDependencies(const SystemConfig& config);

    static std::unique_ptr<SLAMSystem> createMonocular(const SystemConfig& config);

    static std::unique_ptr<SLAMSystem>createMonocularVI(const SystemConfig& config);

    static std::unique_ptr<SLAMSystem>createStereoVI(const SystemConfig& config);
};