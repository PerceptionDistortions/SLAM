#pragma once

#include <string>

struct LoggingConfig
{
    std::string level{"info"};


    struct Modules
    {
        bool sensorManager{true};
        bool bufferManager{true};
        bool frontend{true};
        bool estimator{true};
        bool backend{true};
        bool loopClosure{true};

    } modules;


    bool saveLogs{true};

    std::string path;
};