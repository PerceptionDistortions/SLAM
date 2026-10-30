#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "Configuration/NewConfigLoader.h"
#include "Configuration/NewConfigs/SystemConfigs.h"
#include "Factory/SLAMSystemFactory.h"
#include "SLAMSystem/SLAMSystem.h"

int main()
{
    try
    {
        const std::string configPath = "config/tempconfig.yaml";

        SystemConfigs config = NewConfigLoader::load(configPath);
        std::cout << "Config loaded successfully." << std::endl;

        auto slamSystem = SLAMSystemFactory::create(config);

        if (!slamSystem->init())
        {
            std::cerr << "Failed to initialize SLAM system."
                      << std::endl;

            return EXIT_FAILURE;
        }

        slamSystem->run();

        slamSystem->shutdown();

        std::cout << "\nSLAM finished." << std::endl;

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: "
                  << e.what()
                  << std::endl;

        return EXIT_FAILURE;
    }
}