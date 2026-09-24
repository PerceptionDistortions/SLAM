#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

enum class VisualizerType
{
    None,
    Pangolin
};

inline VisualizerType visualizerTypeFromString(std::string_view value)
{
    if (value == "none")
        return VisualizerType::None;

    if (value == "pangolin")
        return VisualizerType::Pangolin;

    throw std::invalid_argument(
        "Invalid visualizer type: " + std::string(value));
}

inline const char* toString(VisualizerType type)
{
    switch (type)
    {
        case VisualizerType::None:
            return "none";

        case VisualizerType::Pangolin:
            return "pangolin";
    }

    throw std::invalid_argument("Invalid VisualizerType");
}