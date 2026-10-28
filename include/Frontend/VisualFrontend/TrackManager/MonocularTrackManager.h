#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/features2d.hpp>

#include "AliasesEnumsTypes/CommonAliases.h"
#include "AliasesEnumsTypes/TrackState.h"
#include "Frontend/VisualFrontend/TrackManager/ITrackManager.h"
#include "Frontend/VisualFrontend/FeatureTracks/MonocularFeatureTrack.h"
#include"Configuration/NewConfigs/VisualTrackConfig.h"

class MonocularTrackManager final : public ITrackManager
{
public:
    //CONSTRUCTOR
    explicit MonocularTrackManager(VisualTrackConfig& config);

    //DESTRUCTOR
    ~MonocularTrackManager() override = default;

    //PREVENTS COPYING TRACKS AND MANAGER
    MonocularTrackManager(const MonocularTrackManager&) = delete;
    MonocularTrackManager& operator=(const MonocularTrackManager&) = delete;

//TRACK MANAGEMENT FUNCTIONS
public:
    //FOR EACH ITERATION, INIT THE TRACK
    //ALL FEATURES IN CURRENT FRAME GET LINKED TO A TRACK IF EXISTS
    //IF NO TRACK MATCH, CREATE A NEW TENTATIVE TRACK
    //ALSO USED IN FIST FRAME INIT
    void update(
        const std::vector<FeatureObservation>& currentObservations,
        const std::vector<cv::DMatch>& matches,
        const std::vector<std::uint8_t>& geometricInlierMask);

    // SUCCESSFUL TRIANGULATION
    bool markTriangulated(TrackId trackId) override;

    //MAP MANAGER ACCEPTS AND REGISTERS A NEW LANDMARK
    bool associateLandmark(TrackId trackId,LandmarkId landmarkId);

    //LOST TRACKS HANDLING
    bool markTriangulatedLost(TrackId trackId);
    bool recoverTriangulatedLost(TrackId trackId);

    bool markLandmarkLost(TrackId trackId);
    bool recoverLandmarkLost(TrackId trackId);
    
    //REMOVE TRACKS THAT HAVE PASSED A CERTAIN AGE BUT STILL NOT USEFUL
    //APPLIES TO ALL 4 STATES (FROM LOST STATES IN TRIAN AND LANDMARK)
    void prune() override;

//GETTERS
public:
    //GET FEATURE TRACK FROM ID IN INTERFACE FORM
    [[nodiscard]] IFeatureTrack* getTrack(TrackId trackId) noexcept override;
    [[nodiscard]] const IFeatureTrack* getTrack(TrackId trackId) const noexcept override;

    //GET FEATURE TRACK FROM ID IN CONCRETE MONECULAR FORM
    [[nodiscard]] MonocularFeatureTrack* getMonocularTrack(TrackId trackId) noexcept;
    [[nodiscard]] const MonocularFeatureTrack* getMonocularTrack(TrackId trackId) const noexcept;

    //GET ALL TRACK IDs IN VECTOR: ALL AND TYPE WISE
    [[nodiscard]] std::vector<TrackId> getTrackIds(TrackState state) const override;
    [[nodiscard]] std::vector<TrackId> getTrackIds() const override;

    void reset() override;

private:
    TrackId createTrack(const FeatureObservation& observation);
    bool appendObservation(MonocularFeatureTrack& track,const FeatureObservation& observation);

    TrackId nextTrackId_{1};

    // TRACK OWNING, NO LANDMARK OWNING: ID TO TRACK MAP
    std::unordered_map<TrackId,std::unique_ptr<MonocularFeatureTrack>> tracks_;

    // Previous frame observations used for temporal matching.
    std::vector<FeatureObservation> previousObservations_;

private:
    //TRACK ID OF FEATURES IN PREV AND CURR FRAME
    //INDEX OF FEATURES MATCH HERE AND IN KEYPOINTS
    std::vector<TrackId> previousFeatureTrackIds_;
    std::vector<TrackId> currentFeatureTrackIds_;

private:
    //TENTATIVE LIFE CYCLE: FROM CONFIG
    int tentative_max_misses{2};
    int tentative_max_matches{2};
    int tentative_max_age{20}; //same as max observation

    //ACTICE LIFE CYCLE: FROM CONFIG
    int active_max_misses{2};
    int active_max_matches{2};
    int active_max_age{20}; //same as max observation

    //TRIANGULATED TRACK
    int triangulated_max_observation{20};
    int triangulated_max_miss{3};
    int triangulated_max_matches{2};

    //TRIANGULATED LOST TRACK
    int triangulated_lost_max_miss{3};
    int triangulated_lost_max_matches{2};

    //LANDMARKS
    int landmark_max_observation{20};
    int landmark_max_miss{3};

    //LANDMARKS LOST
    int landmark_lost_max_miss{3};
    int landmark_lost_max_matches{2};
};
