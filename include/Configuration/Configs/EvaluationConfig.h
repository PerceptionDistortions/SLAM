#pragma once

#include <string>

struct EvaluationConfig
{
    bool enabled{true};


    struct Metrics
    {
        struct ATE
        {
            bool enabled{true};

        } ate;


        struct RPE
        {
            bool enabled{true};

        } rpe;

    } metrics;


    struct GroundTruth
    {
        bool enabled{true};

        std::string path;

    } groundTruth;
};