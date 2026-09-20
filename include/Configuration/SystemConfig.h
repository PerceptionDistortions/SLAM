#pragma once

#include <string>
#include <vector>


struct SystemConfig
{
    // --------------------------------------------------
    // System
    // --------------------------------------------------

    struct System
    {
        std::string mode;
        std::string visualizer;
    } system;


    // --------------------------------------------------
    // Data source
    // --------------------------------------------------

    struct DataSource
    {
        std::string type;

        struct Dataset
        {
            std::string name;
            std::string path;
        } dataset;

        struct Playback
        {
            bool realtime{false};
            double rate{1.0};
        } playback;

    } dataSource;


    // --------------------------------------------------
    // Sensors
    // --------------------------------------------------

    struct Sensor
    {
        std::string id;
        std::string type;
        std::string source;
        std::string driver;

        struct Config
        {
            std::string stream;
            std::string dataPath;
            std::string csv;

            struct Calibration
            {
                std::string type;
                std::string name;
                std::string file;
            } calibration;

        } config;
    };

    std::vector<Sensor> sensors;


    // --------------------------------------------------
    // Visual frontend
    // --------------------------------------------------

    struct VisualFrontend
    {
        std::string detectorDescriptor;
        std::string featureMatching;
    } visualFrontend;
};