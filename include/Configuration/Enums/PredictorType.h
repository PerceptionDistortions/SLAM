#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

enum class PredictorType
{
    ConstantVelocity,
    IMU
};

inline PredictorType predictorTypeFromString(std::string_view value)
{
    if (value == "constant_velocity")
        return PredictorType::ConstantVelocity;

    if (value == "imu")
        return PredictorType::IMU;

    throw std::invalid_argument(
        "Invalid predictor type: " + std::string(value));
}

inline const char* toString(PredictorType type)
{
    switch (type)
    {
        case PredictorType::ConstantVelocity:
            return "constant_velocity";

        case PredictorType::IMU:
            return "imu";
    }

    throw std::invalid_argument("Invalid PredictorType");
}