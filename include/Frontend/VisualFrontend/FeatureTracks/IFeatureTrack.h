#pragma once

#include <optional>
#include <vector>
#include <Eigen/Core>

#include "AliasesEnumsTypes/CommonAliases.h"
#include "Frontend/VisualFrontend/FeatureTracks/FeatureObservation.h"
#include"AliasesEnumsTypes/TrackState.h"

class IFeatureTrack
{
public:
    virtual ~IFeatureTrack() = default;

    // Identity
    virtual TrackId getId() const noexcept = 0;

    // Lifecycle
    virtual TrackState getState() const noexcept = 0;
    virtual void setState(TrackState state) = 0;

    // Observation history
    virtual const std::vector<FeatureObservation>&
    getObservations() const noexcept = 0;

    virtual bool addObservation(
        const FeatureObservation& observation) = 0;

    // Map association: MapManager owns the landmark.
    virtual std::optional<LandmarkId>
    getLandmarkId() const noexcept = 0;

    virtual void setLandmarkId(LandmarkId landmarkId) = 0;
    virtual void clearLandmarkId() = 0;

    // Tracking quality
    virtual int getConsecutiveInliers() const noexcept = 0;
    virtual void setConsecutiveInliers(int count) = 0;

    virtual int getConsecutiveFailures() const noexcept = 0;
    virtual void setConsecutiveFailures(int count) = 0;

    // Temporal bounds
    virtual FrameId getFirstFrameId() const noexcept = 0;
    virtual FrameId getLastFrameId() const noexcept = 0;

    // Stored 3D position for a triangulated track (world coordinates).
    virtual const std::optional<Eigen::Vector3d>&
    getTriangulatedPointWorld() const noexcept = 0;

    virtual void setTriangulatedPointWorld(
        const Eigen::Vector3d& pointWorld) = 0;

    virtual void clearTriangulatedPointWorld() = 0;
};
