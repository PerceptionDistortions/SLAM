#pragma once
#include<string>

struct SLAMConfigs
{
    //SLAM TYPE
    std::string type{"mono"};

    //MONOUCLAR SLAM
    std::string monoCamId{"cam0"};

    //MONO VI
    std::string monoViCamId{"cam0"};
    std::string monoViImuId{"imu0"};

    //STEREO VI
    std::string stereoViLeftCamId{"cam0"};
    std::string stereoViRightCamId{"cam1"};
    std::string stereoViImuId{"imu0"};

    //VISUALIZER
    std::string visualizer{"pangolin"};

    //CONTROLLED PLAYBACK
    //WILL WORK ONLT FOR DATASETS
    bool keyBasedControlledPlayback{false};
};
