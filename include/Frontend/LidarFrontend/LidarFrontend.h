#pragma once

#include "Estimation/Measurement/Measurement.h"
#include"Frontend/Frontend.h"


#include <memory>

class LidarScan;

class LidarFrontend:public Frontend
{
public:
    virtual ~LidarFrontend() = default;

    virtual std::unique_ptr<Measurement> processLidar(
        const LidarScan& scan) = 0;

    //INIT AND SHUTDOWN ALREADY DEFINED
};