#pragma once
#include<string>

struct PnpConfig{
    std::string pnpStrategy{"epnp"};
    int minimumLandmarkPnp{20};

    int pnpIterationsCount{100};
    double pnpReprojectionThreshold{3.0};
    double pnpConfidence{0.99};
    int pnpMinCorrespondence{6};
    bool pnpRefineWithInliers{true};
};