#pragma once

#include <string>

#include "Configuration/Enums/BackendOptimizerType.h"

struct BackendConfig
{
    bool enabled{true};

    struct Optimization
    {
        bool enabled{true};
        std::string type;
        int windowSize{10};

        struct Optimize
        {
            bool poses{true};
            bool landmarks{true};
            bool intrinsics{false};
            bool extrinsics{false};

        } optimize;


        struct Solver
        {
            BackendOptimizerType type{BackendOptimizerType::Ceres};
            std::string algorithm{"levenberg_marquardt"};
            int maxIterations{20};
        } solver;

    };

    Optimization local;
    Optimization global;

    struct PoseGraph
    {
        bool enabled{true};

        struct Optimize
        {
            bool poses{true};

        } optimize;


        struct Solver
        {
            BackendOptimizerType type{
                BackendOptimizerType::Ceres
            };

            std::string algorithm{
                "levenberg_marquardt"
            };

            int maxIterations{50};

        } solver;

    } poseGraph;
};