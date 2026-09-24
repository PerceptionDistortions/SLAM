#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

enum class BackendOptimizerType
{
    Ceres,
    G2O,
    GTSAM
};

inline BackendOptimizerType backendOptimizerTypeFromString(
    std::string_view value)
{
    if (value == "ceres")
        return BackendOptimizerType::Ceres;

    if (value == "g2o")
        return BackendOptimizerType::G2O;

    if (value == "gtsam")
        return BackendOptimizerType::GTSAM;

    throw std::invalid_argument(
        "Invalid backend optimizer type: " + std::string(value));
}

inline const char* toString(BackendOptimizerType type)
{
    switch (type)
    {
        case BackendOptimizerType::Ceres:
            return "ceres";

        case BackendOptimizerType::G2O:
            return "g2o";

        case BackendOptimizerType::GTSAM:
            return "gtsam";
    }

    throw std::invalid_argument(
        "Invalid BackendOptimizerType");
}