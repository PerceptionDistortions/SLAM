#pragma once

#include <optional>
#include <vector>

#include "AliasesEnumsTypes/CommonAliases.h"
#include "Frontend/VisualFrontend/FeatureTracks/FeatureObservation.h"
#include"AliasesEnumsTypes/TrackState.h"

class IFeatureTrack
{
public:
    virtual ~IFeatureTrack() = default;

    // Identity
    virtual slam::TrackId getId() const noexcept = 0;

    // Lifecycle
    virtual slam::TrackState getState() const noexcept = 0;
    virtual void setState(slam::TrackState state) = 0;

    // Observation history
    virtual const std::vector<FeatureObservation>&
    getObservations() const noexcept = 0;

    virtual bool addObservation(
        const FeatureObservation& observation) = 0;

    // Map association: MapManager owns the landmark.
    virtual std::optional<slam::LandmarkId>
    getLandmarkId() const noexcept = 0;

    virtual void setLandmarkId(slam::LandmarkId landmarkId) = 0;
    virtual void clearLandmarkId() = 0;

    // Tracking quality
    virtual int getConsecutiveInliers() const noexcept = 0;
    virtual void setConsecutiveInliers(int count) = 0;

    virtual int getConsecutiveFailures() const noexcept = 0;
    virtual void setConsecutiveFailures(int count) = 0;

    // Temporal bounds
    virtual slam::FrameId getFirstFrameId() const noexcept = 0;
    virtual slam::FrameId getLastFrameId() const noexcept = 0;
};
