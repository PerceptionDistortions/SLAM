#pragma once

struct MapConfig
{
    struct Landmarks
    {
        bool enabled{true};

    } landmarks;


    struct Keyframes
    {
        bool enabled{true};

    } keyframes;


    struct LocalMap
    {
        bool enabled{true};

    } localMap;


    struct GlobalMap
    {
        bool enabled{true};

    } globalMap;
};