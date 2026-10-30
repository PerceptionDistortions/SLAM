#pragma once

#include<memory>

#include"Configuration/NewConfigs/SystemConfigs.h"
#include "SLAMSystem/SLAMSystem.h"
#include "SLAMSystemDependencies.h"

class SLAMSystemFactory{
public:
    //MAIN CALLS THIS FUNCTION AND THE RETURNED OBJECT IS OWNED BY MAIN: SLAM SYSTEM
    static std::unique_ptr<SLAMSystem> create(const SystemConfigs& config);
    
private:
    static SLAMSystemDependencies createDependencies(const SystemConfigs& config,
    const std::string& configPath);

    static std::unique_ptr<SLAMSystem> createMonocular(const SystemConfigs& config);

    static std::unique_ptr<SLAMSystem>createMonocularVI(const SystemConfigs& config);

    static std::unique_ptr<SLAMSystem>createStereoVI(const SystemConfigs& config);
};