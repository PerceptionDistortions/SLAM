#pragma once

#include <string>

struct LoopClosureConfig
{
    bool enabled{true};


    struct PlaceRecognition
    {
        std::string type;


        struct Vocabulary
        {
            std::string file;

        } vocabulary;


        double similarityThreshold{0.75};

        int minTemporalDistance{30};

    } placeRecognition;


    struct CandidateVerification
    {
        bool enabled{true};


        struct FeatureMatching
        {
            bool enabled{true};

        } featureMatching;


        struct GeometricVerification
        {
            bool enabled{true};

            std::string method;

            int minInliers{30};

            double maxReprojectionError{3.0};

        } geometricVerification;

    } candidateVerification;


    struct Constraint
    {
        bool enabled{true};

        std::string type;

        int minInliers{30};

        double maxReprojectionError{3.0};

    } constraint;


    struct Confirmation
    {
        bool enabled{true};

        int minConsistentMatches{3};

    } confirmation;
};