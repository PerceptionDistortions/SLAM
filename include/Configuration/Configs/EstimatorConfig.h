#pragma once

#include <string>

#include "Configuration/Enums/EstimatorType.h"
#include "Configuration/Enums/PredictorType.h"
#include "Configuration/Enums/BackendOptimizerType.h"

struct EstimatorConfig
{
    EstimatorType type{
        EstimatorType::Filter
    };


    // ========================================================
    // FILTER
    // ========================================================

    struct Filter
    {
        std::string type{"esekf"};


        struct Prediction
        {
            PredictorType type{
                PredictorType::ConstantVelocity
            };


            struct ConstantVelocity
            {
                bool enabled{true};

            } constantVelocity;


            struct IMU
            {
                bool enabled{false};

            } imu;

        } prediction;


        struct Correction
        {
            struct Visual
            {
                bool enabled{true};

                std::string type;

            } visual;

        } correction;

    } filter;


    // ========================================================
    // OPTIMIZATION
    // ========================================================

    struct Optimization
    {
        bool enabled{false};

        std::string type{"sliding_window"};

        int windowSize{10};


        struct Factors
        {
            bool visual{true};
            bool imu{false};

        } factors;


        struct Marginalization
        {
            bool enabled{true};

        } marginalization;


        struct Solver
        {
            BackendOptimizerType type{
                BackendOptimizerType::Ceres
            };

            std::string algorithm{
                "levenberg_marquardt"
            };

            int maxIterations{20};

        } solver;

    } optimization;
};