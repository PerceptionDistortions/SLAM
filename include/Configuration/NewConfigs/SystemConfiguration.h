#pragma once

#include <vector>
#include<string>

#include "Configuration/Enums/SLAMType.h"
#include"Configuration/NewConfigs/VisualFrontendConfiguration.h"
#include"Configuration/NewConfigs/SensorConfiguration.h"

struct SystemConfiguration{
    SLAMType slamType;

    //SENSOR NAMES FOR EACH TYPE OF SLAM
    std::string monocular_camera;

    std::string monocularvi_camera;
    std::string monocularvi_imu;

    std::string stereovi_leftcamera;
    std::string stereovi_rightcamera;
    std::string stereovi_imu;

    //OTHER CONFIGURATIONS
    
    VisualFrontendConfiguration visualFrontendConfig;
};