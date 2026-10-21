
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

class MonocularTrackManager final : public ITrackManager
{
public:
    //CONSTRUCTOR
    explicit MonocularTrackManager(int maxConsecutiveFailures = 2,std::size_t maxObservationHistory = 30);

    //DESTRUCTOR
    ~MonocularTrackManager() override = default;

    MonocularTrackManager(const MonocularTrackManager&) = delete;
    MonocularTrackManager& operator=(const MonocularTrackManager&) = delete;

    //TRACK LOOKUP
    [[nodiscard]] IFeatureTrack* getTrack(TrackId trackId) noexcept override;
    [[nodiscard]] const IFeatureTrack* getTrack(TrackId trackId) const noexcept override;
    [[nodiscard]] MonocularFeatureTrack* getMonocularTrack(TrackId trackId) noexcept;
    [[nodiscard]] const MonocularFeatureTrack* getMonocularTrack(TrackId trackId) const noexcept;
    [[nodiscard]] std::vector<TrackId> getTrackIds(TrackState state) const override;
    [[nodiscard]] std::vector<TrackId> getTrackIds() const override;

    //TEMPORAL TRACKING IF REQUIRED: TODO

    void update(
        const std::vector<FeatureObservation>& currentObservations,
        const std::vector<cv::DMatch>& matches,
        const std::vector<std::uint8_t>& geometricInlierMask);

    // -------------------------------------------------
    // Track lifecycle
    // -------------------------------------------------

    // Called after successful triangulation.
    bool markTriangulated(TrackId trackId) override;

    // Called after MapManager accepts and registers
    // the corresponding landmark.
    bool associateLandmark(TrackId trackId,LandmarkId landmarkId);

    // Mark a track as lost when appropriate.
    bool markLost(TrackId trackId);

    // Remove stale tracks according to lifecycle policy.
    void prune() override;

    void reset() override;

private:
    TrackId createTrack(const FeatureObservation& observation);
    bool appendObservation(MonocularFeatureTrack& track,const FeatureObservation& observation);

    int maxConsecutiveFailures_;
    std::size_t maxObservationHistory_;

    TrackId nextTrackId_{1};

    // This manager owns tracks, never Landmark objects.
    std::unordered_map<TrackId,std::unique_ptr<MonocularFeatureTrack>> tracks_;

    // Index-aligned associations for consecutive frames.
    std::vector<TrackId> previousFeatureTrackIds_;
    std::vector<TrackId> currentFeatureTrackIds_;

    // Previous frame observations used for temporal matching.
    std::vector<FeatureObservation> previousObservations_;
};
