#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

enum class DataSourceType
{
    Dataset,
    Physical
};

inline DataSourceType dataSourceTypeFromString(
    std::string_view value)
{
    if (value == "dataset")
        return DataSourceType::Dataset;

    if (value == "physical")
        return DataSourceType::Physical;

    throw std::invalid_argument(
        "Invalid data source type: " +
        std::string(value));
}

inline const char* toString(DataSourceType type)
{
    switch (type)
    {
        case DataSourceType::Dataset:
            return "dataset";

        case DataSourceType::Physical:
            return "physical";
    }

    throw std::invalid_argument(
        "Invalid DataSourceType");
}