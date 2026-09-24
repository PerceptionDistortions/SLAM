#pragma once

#include <string>

struct OutputConfig
{
    struct Trajectory
    {
        bool enabled{true};

        std::string format{"tum"};

        std::string path;

    } trajectory;


    struct Landmarks
    {
        bool enabled{true};

        std::string path;

    } landmarks;


    struct Metrics
    {
        bool enabled{true};

        std::string path;

    } metrics;


    struct Covariance
    {
        bool enabled{true};

    } covariance;
};