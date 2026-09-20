#pragma once

#include "Configuration/SystemConfig.h"

#include <string>


class ConfigLoader
{
public:

    static SystemConfig load(
        const std::string& filename);
};