#pragma once

#include<string>

struct FilterEstimatorConfig
{
    //INITIAL COVARIANCE
    double position_sd{0.0};
    double velocity_sd{0.5};
    double orientation_sd{0.0};
    double acc_bias_sd{0.0};
    double gyro_bias_sd{0.0};

    // PROCESS NOISE SD
    double acceleration_noise_sd{0.05};
    double orientation_noise_sd{0.01};
    double acc_bias_noise_sd{0.001};
    double gyro_bias_noise_sd{0.001};

    // MEASUREMENT NOISE SD
    double visual_position_noise_sd{0.05};
    double visual_orientation_noise_sd{0.02};

};