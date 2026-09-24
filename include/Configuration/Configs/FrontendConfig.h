#pragma once

#include <string>

struct FrontendConfig
{
    struct Visual
    {
        bool enabled{true};


        struct DetectorDescriptor
        {
            std::string type;

            int nFeatures{2000};
            double scaleFactor{1.2};
            int nLevels{8};

        } detectorDescriptor;


        struct Preprocessing
        {
            bool grayscale{true};
            bool undistort{true};

        } preprocessing;


        struct FeatureMatching
        {
            std::string type;
            std::string descriptor;

            double ratioTest{0.75};
            bool crossCheck{false};

        } featureMatching;


        struct GeometricVerification
        {
            bool enabled{true};

            std::string method;

            double maxReprojectionError{3.0};

        } geometricVerification;


        struct MotionEstimation
        {
            bool enabled{true};

        } motionEstimation;


        struct Triangulation
        {
            bool enabled{true};

            double minParallax{1.0};
            double maxReprojectionError{3.0};

        } triangulation;


        struct Tracking
        {
            bool enabled{true};

            int minTrackedFeatures{100};

        } tracking;


        struct LandmarkManagement
        {
            bool enabled{true};

        } landmarkManagement;


        struct KeyframeSelection
        {
            bool enabled{true};

        } keyframeSelection;

    } visual;


    struct IMU
    {
        bool enabled{false};


        struct Preprocessing
        {
            bool removeBias{true};
            std::string noiseModel;

        } preprocessing;


        struct Preintegration
        {
            bool enabled{false};

        } preintegration;

    } imu;


    struct LiDAR
    {
        bool enabled{false};

    } lidar;
};