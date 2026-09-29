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

int main()
{
    try
    {
        // ============================================================
        // GET CONFIG PATH
        // STORE DATA IN SYSTEM CONFIG AS OWNER
        // ============================================================

        const std::string configPath =
            "config/system.yaml";

        SystemConfig config =
            ConfigLoader::load(configPath);

        std::cout
            << "Config loaded successfully."
            << std::endl;


        // ============================================================
        // SENSOR MANAGER
        // SENSOR CONFIGURATION
        // ============================================================

        SensorManager sensorManager;

        if (!sensorManager.loadConfig(configPath))
        {
            std::cerr
                << "Failed to load SensorManager configuration."
                << std::endl;

            return EXIT_FAILURE;
        }


        // ============================================================
        // INITIALIZE SENSORS
        // ============================================================

        if (!sensorManager.init())
        {
            std::cerr
                << "Failed to initialize SensorManager."
                << std::endl;

            return EXIT_FAILURE;
        }


        // ============================================================
        // START SENSORS
        // ============================================================

        if (!sensorManager.start())
        {
            std::cerr
                << "Failed to start SensorManager."
                << std::endl;

            return EXIT_FAILURE;
        }


        // ============================================================
        // CHRONOLOGICAL SENSOR STREAM
        // ============================================================

        std::cout
            << "\n=== Chronological Sensor Stream ==="
            << std::endl;

        SensorManager::Data data;


        // ============================================================
        // CONSUME SENSOR DATA
        // ============================================================

        /*
         * popNext() is non-blocking.
         *
         * Therefore, false does NOT necessarily mean that
         * the dataset has finished.
         *
         * It can simply mean that no chronologically releasable
         * measurement is available yet.
         *
         * SensorManager::isRunning() determines whether the
         * producer threads are still running.
         */

        while (sensorManager.isRunning())
        {
            if (sensorManager.popNext(data))
            {
                std::visit(
                    [](const auto& sensorData)
                    {
                        using T =
                            std::decay_t<
                                decltype(sensorData)>;

                        if constexpr (
                            std::is_same_v<
                                T,
                                CameraData>)
                        {
                            std::cout
                                << "[CAMERA]"
                                << " sensor="
                                << sensorData.sensor_id
                                << " timestamp="
                                << sensorData.timestamp
                                << std::endl;
                        }
                        else if constexpr (
                            std::is_same_v<
                                T,
                                ImuData>)
                        {
                            std::cout
                                << "[IMU]"
                                << " sensor="
                                << sensorData.sensor_id
                                << " timestamp="
                                << sensorData.timestamp
                                << std::endl;
                        }
                    },
                    data);
            }
            else
            {
                /*
                 * No data available right now.
                 *
                 * Give the DatasetPlayer threads some time
                 * to produce the next measurements.
                 */

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1));
            }
        }


        // ============================================================
        // DRAIN REMAINING BUFFERED DATA
        // ============================================================

        /*
         * The producer threads may have finished while there
         * are still measurements remaining inside BufferManager.
         *
         * Drain those measurements before shutting down.
         */

        while (sensorManager.popNext(data))
        {
            std::visit(
                [](const auto& sensorData)
                {
                    using T =
                        std::decay_t<
                            decltype(sensorData)>;

                    if constexpr (
                        std::is_same_v<
                            T,
                            CameraData>)
                    {
                        std::cout
                            << "[CAMERA]"
                            << " sensor="
                            << sensorData.sensor_id
                            << " timestamp="
                            << sensorData.timestamp
                            << std::endl;
                    }
                    else if constexpr (
                        std::is_same_v<
                            T,
                            ImuData>)
                    {
                        std::cout
                            << "[IMU]"
                            << " sensor="
                            << sensorData.sensor_id
                            << " timestamp="
                            << sensorData.timestamp
                            << std::endl;
                    }
                },
                data);
        }


        // ============================================================
        // STOP SENSOR MANAGER
        // ============================================================

        sensorManager.stop();


        std::cout
            << "\nSensor stream finished."
            << std::endl;


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
}