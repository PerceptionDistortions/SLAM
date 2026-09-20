#include <iostream>
#include <type_traits>

#include <thread>
#include <chrono>
#include<memory>

#include"Sensors/SensorManager/SensorManager.h"
#include "Configuration/ConfigLoader.h"

int main(){

    try{
        //LOAD SYSTEM CONFIGURATION: PARSE IN FILE
        SystemConfig config=ConfigLoader::load("config/system.yaml");

        //CREATE CONCRETE SLAM SYSTEM
    }
    catch(const std::exception& e){

    }
    return 0;



    //SENSOR MANAGER: INIT AND START
    // SensorManager sensorManager;
    // if (!sensorManager.loadConfig("config/sensors/system.yaml")) return -1;
    // if(!sensorManager.init()) return -1;
    // if(!sensorManager.start()) return -1;

    // //READ SENSOR DATA
    // SensorManager::Data data;
    // while (sensorManager.isRunning() || !sensorManager.popNext(data))
    // {
    //     if (sensorManager.popNext(data))
    //     {
    //         std::visit(
    //             [](const auto& sensorData)
    //             {
    //                 using T = std::decay_t<decltype(sensorData)>;

    //                 std::cout << "--------------------------------\n";
    //                 std::cout << "Sensor ID : " << sensorData.sensor_id << '\n';
    //                 std::cout << "Timestamp : " << sensorData.timestamp << '\n';

    //                 if constexpr (std::is_same_v<T, ImuData>)
    //                 {
    //                     std::cout << "Type      : IMU\n";
    //                     std::cout << "Gyro      : "
    //                             << sensorData.gyro_x << ", "
    //                             << sensorData.gyro_y << ", "
    //                             << sensorData.gyro_z << '\n';
    //                     std::cout << "Accel     : "
    //                             << sensorData.accel_x << ", "
    //                             << sensorData.accel_y << ", "
    //                             << sensorData.accel_z << '\n';
    //                 }
    //                 else if constexpr (std::is_same_v<T, CameraData>)
    //                 {
    //                     std::cout << "Type      : Camera\n";
    //                     std::cout << "Image     : "
    //                             << sensorData.image.cols << " x "
    //                             << sensorData.image.rows << '\n';
    //                 }
    //             },
    //             data);
    //     }
    //     else
    //     {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //     }
    }

    //SLAM

    //STOP THE SENSOR MANAGER
    sensorManager.stop();
    return 0;
}