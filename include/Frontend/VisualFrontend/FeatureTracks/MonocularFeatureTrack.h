#pragma once

#include <cstddef>

#include "Frontend/VisualFrontend/FeatureTracks/IFeatureTrack.h"

//A SINGLE MONOCULAR TRACK REPRESENTATION
class MonocularFeatureTrack final : public IFeatureTrack
{
public:
    //CONSTRUCTOR
    explicit MonocularFeatureTrack(TrackId id,const FeatureObservation& initialObservation);

    //GET THE TRACK ID
    TrackId getId() const noexcept override;

    //GET AND SET STATE: TENTATIVE, ACTIVE, ETC.
    TrackState getState() const noexcept override;
    void setState(TrackState state) override;

    //GET COMPLETE OBSERVATIONS AND ADD OBSERVATIONS
    const std::vector<FeatureObservation>& getObservations() const noexcept override;
    bool addObservation(const FeatureObservation& observation) override;

    // LANDMARK ID WILL BE SEPARATE FROM FRAME ID
    std::optional<LandmarkId> getLandmarkId() const noexcept override;
    void setLandmarkId(LandmarkId landmarkId) override;
    void clearLandmarkId() override;

    // Tracking quality
    int getConsecutiveInliers() const noexcept override;
    void setConsecutiveInliers(int count) override;

    int getConsecutiveFailures() const noexcept override;
    void setConsecutiveFailures(int count) override;

    // Temporal bounds
    FrameId getFirstFrameId() const noexcept override;
    FrameId getLastFrameId() const noexcept override;

    void trimObservationHistory();

    //TRIANGULATION TRACK UTILITY
    // Stored triangulated 3D point in world coordinates
    const std::optional<Eigen::Vector3d>&
    getTriangulatedPointWorld() const noexcept override;
    void setTriangulatedPointWorld(const Eigen::Vector3d& pointWorld) override;
    void clearTriangulatedPointWorld() override;

public:
    //AGE HANDLING
    int getTrackAge() const noexcept;
    void incrementTrackAge() noexcept;
    void resetTrackAge() noexcept;

private:
    //TRACK AND LANDMARK ID
    TrackId id_{0};
    std::optional<LandmarkId> landmarkId_;

    //TRACK STATE
    TrackState state_{TrackState::Tentative};

    //ALL FEATURE OBSERVATIONS: CAMERA, FRAME, FEATURE, PIXEL
    //MAX COUNT WILL DEPEND ON CONFIGURATION
    std::vector<FeatureObservation> observations_;

    //CONSECUTIVE MATCHES AND MISSES
    int consecutiveInliers_{0};
    int consecutiveFailures_{0};
    int track_age{0};

    //IMPORTANT FRAMES IDs WITH THIS TRACK
    FrameId firstFrameId_{0};
    FrameId prevFrameId_{0};
    FrameId currentFrameId{0};

    //IF TRIANGULATED, STORE 3D POINTS
    std::optional<Eigen::Vector3d> triangulatedPointWorld_; //3D world point
};
