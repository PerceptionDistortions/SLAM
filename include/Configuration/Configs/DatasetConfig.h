#pragma once

#include <string>

struct DatasetConfig
{
    std::string name;
    std::string path;

    struct Playback
    {
        bool realtime{false};
        double rate{1.0};

    } playback;
};