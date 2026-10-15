#pragma once

struct PnPConfig
{
    int iterationsCount = 100;
    float reprojectionThreshold = 3.0f;
    double confidence = 0.99;
    int minCorrespondences = 6;
    bool refineWithInliers = true;
};

