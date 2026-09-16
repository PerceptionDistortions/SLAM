#pragma once

#include <cstdint>
#include<string>

class ImuData
{
public:
    std::string sensor_id;
    int64_t timestamp = 0;

    double accel_x = 0.0;
    double accel_y = 0.0;
    double accel_z = 0.0;

    double gyro_x = 0.0;
    double gyro_y = 0.0;
    double gyro_z = 0.0;
};