#pragma once

#include<string>

struct VisualFrontendConfig
{
    //DETECTOR AND DESCRIPTOR
    std::string detector_descriptor;

    //ORB DETECTOR AND DESCRIPTOR
    int orb_featureCount{2000};
    double orb_scale_factor{1.2};
    int orb_levels{8};

    //FEATURE MATCHING: BF, NONE
    std::string feature_matcher;
    std::string bf_norm;

    //FEATURE TRACKING: OPTICAL, NONE
    std::string feature_tracker;

    //MOTION BLUR ALGORITHM: LAPLACIAN
    std::string motion_blur;

    //LIGHT EXPOSURE ALGORITHM STRATEGY
    std::string light_exposure;

    //LOWE RATIO
    double lowe_ratio{0.7};

    //CROSS CHECK ALGORITHM STRATEGY: MUTUAL_BEST
    std::string cross_check;

    //DISTANCE FILTERING ALGORITHM
    std::string distance_filtering;
};
