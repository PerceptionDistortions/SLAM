#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#include "Configuration/ConfigLoader.h"
#include "Configuration/Configs/SystemConfig.h"
#include "Factory/SLAMSystemFactory.h"
#include"SLAMSystem/SLAMSystem.h"

int main()
{
    try
    {
        //CONFIG FILE READ
        const std::string configPath ="config/system.yaml";
        SystemConfig config =ConfigLoader::load(configPath);

        //VALID CONFIG: SUCCESS
        std::cout<< "Config loaded successfully."<< std::endl;

        //SLAM SYSTEM: MONOCULAR
        //CHECK IF CREATED
        auto slamSystem=SLAMSystemFactory::create(config);
        if(!slamSystem){
            std::cerr<< "Failed to create SLAM system."<< std::endl;
            return EXIT_FAILURE;
        }

        //INIT THE SLAM SYSTEM
        //START THE SLAM SYSTEM
        if(!slamSystem->init()){
            std::cerr<< "Failed to initialize SLAM system."<< std::endl;
            return EXIT_FAILURE;
        }

        //RUN SLAM: USE SLAM UPDATE
        slamSystem->run();

        //SHUTDOWN SLAM
        slamSystem->shutdown();

        //SLAM FINISHED
        std::cout<< "\nSLAM finished."<< std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "Fatal error: "
            << e.what()
            << std::endl;

        return EXIT_FAILURE;
    }
    

    //     //SENSOR MANAGER AND CONFIG
    //     //DEFAULT CONSTRUCTOR CREATES BOTH BUFFER AND CALIBRATOR
    //     SensorManager sensorManager;
    //     if (!sensorManager.loadConfig(configPath))
    //     {
    //         std::cerr<< "Failed to load SensorManager configuration."<< std::endl;
    //         return EXIT_FAILURE;
    //     }

    //     //INIT SENSORS: CONTEXT-> DRIVERS
    //     if (!sensorManager.init())
    //     {
    //         std::cerr<< "Failed to initialize SensorManager."<< std::endl;
    //         return EXIT_FAILURE;
    //     }

    //     //START SENSOR-> CONTEXT-> DRIVER START
    //     if (!sensorManager.start())
    //     {
    //         std::cerr<< "Failed to start SensorManager."<< std::endl;
    //         return EXIT_FAILURE;
    //     }

    //     std::cout<< "\n=== Chronological Sensor Stream ==="<< std::endl;
    //     SensorManager::Data data;


    //     // ============================================================
    //     // CONSUME SENSOR DATA
    //     // ============================================================

    //     /*
    //      * popNext() is non-blocking.
    //      *
    //      * Therefore, false does NOT necessarily mean that
    //      * the dataset has finished.
    //      *
    //      * It can simply mean that no chronologically releasable
    //      * measurement is available yet.
    //      *
    //      * SensorManager::isRunning() determines whether the
    //      * producer threads are still running.
    //      */

    //     while (sensorManager.isRunning())
    //     {
    //         if (sensorManager.popNext(data))
    //         {
    //             std::visit(
    //                 [](const auto& sensorData)
    //                 {
    //                     using T =std::decay_t<decltype(sensorData)>;
    //                     if constexpr (std::is_same_v<T,CameraData>)
    //                     {
    //                         std::cout
    //                             << "[CAMERA]"
    //                             << " sensor="
    //                             << sensorData.sensor_id
    //                             << " timestamp="
    //                             << sensorData.timestamp
    //                             << std::endl;
    //                     }
    //                     else if constexpr (std::is_same_v<T,ImuData>)
    //                     {
    //                         std::cout
    //                             << "[IMU]"
    //                             << " sensor="
    //                             << sensorData.sensor_id
    //                             << " timestamp="
    //                             << sensorData.timestamp
    //                             << std::endl;
    //                     }
    //                 },
    //                 data);
    //         }
    //         else
    //         {
    //             //GIVE SOME BREAK: WAIT
    //             std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //         }
    //     }

    //     //POP RATE OF BUFFER IS SLOWER THAN THE PRODUCER
    //     while (sensorManager.popNext(data))
    //     {
    //         std::visit(
    //             [](const auto& sensorData)
    //             {
    //                 using T =
    //                     std::decay_t<
    //                         decltype(sensorData)>;

    //                 if constexpr (
    //                     std::is_same_v<
    //                         T,
    //                         CameraData>)
    //                 {
    //                     std::cout
    //                         << "[CAMERA]"
    //                         << " sensor="
    //                         << sensorData.sensor_id
    //                         << " timestamp="
    //                         << sensorData.timestamp
    //                         << std::endl;
    //                 }
    //                 else if constexpr (
    //                     std::is_same_v<
    //                         T,
    //                         ImuData>)
    //                 {
    //                     std::cout
    //                         << "[IMU]"
    //                         << " sensor="
    //                         << sensorData.sensor_id
    //                         << " timestamp="
    //                         << sensorData.timestamp
    //                         << std::endl;
    //                 }
    //             },
    //             data);
    //     }


    //     // ============================================================
    //     // STOP SENSOR MANAGER
    //     // ============================================================

    //     sensorManager.stop();


    //     std::cout
    //         << "\nSensor stream finished."
    //         << std::endl;


    //     return EXIT_SUCCESS;
    // }
    // catch (const std::exception& e)
    // {
    //     std::cerr
    //         << "Fatal error: "
    //         << e.what()
    //         << std::endl;

    //     return EXIT_FAILURE;
    // }
}