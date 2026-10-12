#pragma once
#include<string>

struct VisualFrontendConfiguration{
    //PREPROCESSING
    std::string motion_blur; //Motion blur
    std::string light_exposure; //light exposure

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

    //DESCRIPTOR FILTERING
    double lowe_ratio{0.7}; //Lowe ratio
    std::string cross_check; //cross check
    std::string distance_filtering; //distance filtering

    //RANSAC
    double recover_pose_distance_threshold;
    int minimum_landmark_pnp;
};