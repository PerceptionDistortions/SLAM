#pragma once

#include "Estimation/Measurement/Measurement.h"
#include"Frontend/Frontend.h"


#include <memory>

class LidarScan;

class LidarFrontend:public Frontend
{
public:
    virtual ~LidarFrontend() = default;

    virtual std::unique_ptr<Measurement> process(
        const LidarScan& scan) = 0;
};