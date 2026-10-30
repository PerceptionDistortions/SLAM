#pragma once
#include<string>

struct VisualFrontendConfigs
{
    // PREPROCESSING
    std::string motionBlur{"laplacian"};
    std::string lightExposure{"clahe"};

    double claheCliplimit{2.0};
    int claheTileLimitSize{8};

    double laplacianVarianceThreshold{100.0};

    // FEATURE DETECTION AND MATCHING
    std::string detectionDescription{"orb"};
    std::string featureMatching{"bf"};
    std::string featureTracking{"none"};

    // ORB PARAMETERS
    int orbFeatureCount{2000};
    double orbScaleFactor{1.2};
    int orbLevels{8};

    // DESCRIPTOR MATCHING AND FILTERING
    double loweRatio{0.7};
    std::string crossCheck{"mutual_best"};
    std::string distanceFiltering{"fixed_distance"};

    //FIXED DISTANCE
    float fixedDistanceFilterMaxDistance{60};

    // MOTION ESTIMATION
    double recoverPoseDistanceThreshold{1e6};

    // MOTION GATING
    int eRansacMatchCount{30};
    int pnpRansacCount{5};
};
