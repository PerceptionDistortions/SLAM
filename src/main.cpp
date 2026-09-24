#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "Configuration/ConfigLoader.h"
#include "Configuration/Configs/SystemConfig.h"
#include "Factory/SLAMSystemFactory.h"

int main(){
    try
    {
        const std::string configPath = "config/system.yaml";

        SystemConfig config = ConfigLoader::load(configPath);

        std::cout << "Config loaded successfully." << std::endl;

        SensorManager sensorManager;

        if (!sensorManager.loadConfig(configPath))
        {
            std::cerr << "Failed to load SensorManager configuration." << std::endl;
            return EXIT_FAILURE;
        }

        if (!sensorManager.init())
        {
            std::cerr << "Failed to initialize SensorManager." << std::endl;
            return EXIT_FAILURE;
        }

        if (!sensorManager.start())
        {
            std::cerr << "Failed to start SensorManager." << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << "\n=== Chronological Sensor Stream ===" << std::endl;

        SensorManager::Data data;

        while (sensorManager.popNext(data))
        {
            std::visit([](const auto& sensorData)
            {
                using T = std::decay_t<decltype(sensorData)>;

                if constexpr (std::is_same_v<T, CameraData>)
                {
                    std::cout << "[CAMERA]"
                              << " timestamp=" << sensorData.timestamp
                              << std::endl;
                }
                else if constexpr (std::is_same_v<T, ImuData>)
                {
                    std::cout << "[IMU]"
                              << " timestamp=" << sensorData.timestamp
                              << std::endl;
                }

            }, data);
        }

        sensorManager.stop();

        std::cout << "\nSensor stream finished." << std::endl;

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}