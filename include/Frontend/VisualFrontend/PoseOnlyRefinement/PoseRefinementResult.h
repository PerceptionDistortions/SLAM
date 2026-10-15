
#pragma once

#include <string>

struct PoseRefinementResult
{
    bool success = false;
    int numCorrespondences = 0;

    // Define both errors consistently, preferably in pixels.
    double initialReprojectionError = 0.0;
    double finalReprojectionError = 0.0;

    std::string message;
};
