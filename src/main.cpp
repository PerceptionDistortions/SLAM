#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "Configuration/ConfigLoader.h"
#include "Configuration/Configs/SystemConfig.h"
#include "Factory/SLAMSystemFactory.h"

int main(){
    //CONFIGURATION LOADING
    const std::string configPath="config/system.yaml";
    SystemConfig config=ConfigLoader::load(configPath);
    std::cout<<"Config loaded succesfully."<<std::endl;
    std::cout
            << "SLAM mode: "
            << static_cast<int>(config.system.mode)
            << std::endl;

        std::cout
            << "Backend enabled: "
            << std::boolalpha
            << config.backend.enabled
            << std::endl;

        std::cout
            << "Loop closure enabled: "
            << config.loopClosure.enabled
            << std::endl;

    return 0;
}