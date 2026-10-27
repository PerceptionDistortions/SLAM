#pragma once
#include<string>

struct VisualTrackConfig{
    //TENTATIVE TRACKS
    int tentative_max_misses{2};
    int tentative_max_age{2};

    //ACTIVE TRACKS
    int active_max_misses{2};
    int active_max_age{2};
};