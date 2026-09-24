#pragma once

#include <string>

#include "Configuration/Enums/DataSourceType.h"
#include "DatasetConfig.h"
#include "PhysicalDataSourceConfig.h"

struct DataSourceConfig
{
    std::string id;

    DataSourceType type{
        DataSourceType::Dataset
    };

    DatasetConfig dataset;

    PhysicalDataSourceConfig physical;
};