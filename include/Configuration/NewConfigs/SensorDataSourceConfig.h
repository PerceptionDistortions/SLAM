#pragma once
#include<string>

struct SensorDataSourceConfig
{
    std::string type{"dataset"};
    std::string datasetId{"euroc"};
    std::string datasetPath{"datasets/euroc"};
    double playbackRate{1.0};
};