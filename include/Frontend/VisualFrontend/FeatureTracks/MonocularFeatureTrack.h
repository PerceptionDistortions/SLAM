#pragma once

#include "Frontend/VisualFrontend/FeatureTracks/IFeatureTrack.h"

class MonocularFeatureTrack final : public IFeatureTrack
{
public:
    explicit MonocularFeatureTrack(
        slam::TrackId id,
        const FeatureObservation& initialObservation);

    // Identity
    slam::TrackId getId() const noexcept override;

    // Lifecycle
    slam::TrackState getState() const noexcept override;
    void setState(slam::TrackState state) override;

    // Observation history
    const std::vector<FeatureObservation>&
    getObservations() const noexcept override;

    bool addObservation(
        const FeatureObservation& observation) override;

    // Landmark association
    std::optional<slam::LandmarkId>
    getLandmarkId() const noexcept override;

    void setLandmarkId(slam::LandmarkId landmarkId) override;
    void clearLandmarkId() override;

    // Tracking quality
    int getConsecutiveInliers() const noexcept override;
    void setConsecutiveInliers(int count) override;

    int getConsecutiveFailures() const noexcept override;
    void setConsecutiveFailures(int count) override;

    // Temporal bounds
    slam::FrameId getFirstFrameId() const noexcept override;
    slam::FrameId getLastFrameId() const noexcept override;

private:
    slam::TrackId id_{0};

    slam::TrackState state_{slam::TrackState::Tentative};

    std::vector<FeatureObservation> observations_;

    std::optional<slam::LandmarkId> landmarkId_;

    int consecutiveInliers_{0};
    int consecutiveFailures_{0};

    slam::FrameId firstFrameId_{0};
    slam::FrameId lastFrameId_{0};
};
