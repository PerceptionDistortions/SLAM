#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

enum class EstimatorType
{
    Filter,
    Optimization
};

inline EstimatorType estimatorTypeFromString(std::string_view value)
{
    if (value == "filter")
        return EstimatorType::Filter;

    if (value == "optimization")
        return EstimatorType::Optimization;

    throw std::invalid_argument(
        "Invalid estimator type: " + std::string(value));
}

inline const char* toString(EstimatorType type)
{
    switch (type)
    {
        case EstimatorType::Filter:
            return "filter";

        case EstimatorType::Optimization:
            return "optimization";
    }

    throw std::invalid_argument("Invalid EstimatorType");
}