#pragma once
#include<string>

struct VisualTrackConfig{
    //TENTATIVE TRACKS
    int tentative_max_match{2};
    int tentative_max_misses{2};
    int tentative_max_age{20};

    //ACTIVE TRACKS
    int active_max_misses{2};
    int active_max_age{20};
    int active_max_matches{2};

    //TRIANGULATED TRACKS
    int triangulated_max_observations{20};
    int triangulated_max_matches{2};
    int triangulated_max_misses{3};

    //TRIANGULATED LOST TRACKS
    int triangulated_lost_max_miss{5};
    int triangulated_lost_max_match{2};

    //LANDMARKS
    int landmark_max_observations{20};
    int landmark_max_misses{3};

    //LANDMARKS LOST TRACKS
    int landmark_lost_max_misses{3};
    int landmark_lost_max_matches{2};
};

  

