
#include "Frontend/VisualFrontend/FeatureTracks/MonocularFeatureTrack.h"

#include <algorithm>
#include <cstddef>

MonocularFeatureTrack::MonocularFeatureTrack(
    TrackId id,
    const FeatureObservation& initialObservation)
    : id_(id),
      state_(TrackState::Tentative),
      observations_{initialObservation},
      firstFrameId_(initialObservation.frameId),
      prevFrameId_(initialObservation.frameId),
      currentFrameId(initialObservation.frameId)
{
}

TrackId MonocularFeatureTrack::getId() const noexcept
{
    return id_;
}

TrackState MonocularFeatureTrack::getState() const noexcept
{
    return state_;
}

void MonocularFeatureTrack::setState(TrackState state)
{
    state_ = state;
}

const std::vector<FeatureObservation>&
MonocularFeatureTrack::getObservations() const noexcept
{
    return observations_;
}

//IF A FEATURE IS MACTCHED IN A TRACK, ADD AN OBSERVATION
bool MonocularFeatureTrack::addObservation(
    const FeatureObservation& observation)
{
    // Reject duplicate or out-of-order frame observations.
    if (!observations_.empty() &&
        observation.frameId <= observations_.back().frameId)
        return false;

    prevFrameId_ = currentFrameId;
    currentFrameId = observation.frameId;

    observations_.push_back(observation);
    return true;
}


std::optional<LandmarkId>MonocularFeatureTrack::getLandmarkId() const noexcept
{
    return landmarkId_;
}

void MonocularFeatureTrack::setLandmarkId(LandmarkId landmarkId)
{
    landmarkId_ = landmarkId;
}

void MonocularFeatureTrack::clearLandmarkId()
{
    landmarkId_.reset();
}

int MonocularFeatureTrack::getConsecutiveInliers() const noexcept
{
    return consecutiveInliers_;
}

void MonocularFeatureTrack::setConsecutiveInliers(int count)
{
    consecutiveInliers_ = std::max(0, count);
}

int MonocularFeatureTrack::getConsecutiveFailures() const noexcept
{
    return consecutiveFailures_;
}

void MonocularFeatureTrack::setConsecutiveFailures(int count)
{
    consecutiveFailures_ = std::max(0, count);
}

FrameId MonocularFeatureTrack::getFirstFrameId() const noexcept
{
    return firstFrameId_;
}

FrameId MonocularFeatureTrack::getLastFrameId() const noexcept
{
    return currentFrameId;
}


void MonocularFeatureTrack::trimObservationHistory()
{
    std::size_t maxObservations = 0;

    switch (state_)
    {
        case TrackState::Triangulated:
            maxObservations = 20;
            break;

        case TrackState::Landmark:
            maxObservations = 20;
            break;

        default:
            // Tentative and active tracks are managed by max age.
            return;
    }

    if (observations_.size() <= maxObservations)
        return;

    const std::size_t excess =
        observations_.size() - maxObservations;

    observations_.erase(
        observations_.begin(),
        observations_.begin() +
            static_cast<
                std::vector<FeatureObservation>::difference_type>(
                    excess));
}


const std::optional<Eigen::Vector3d>&
MonocularFeatureTrack::getTriangulatedPointWorld() const noexcept
{
    return triangulatedPointWorld_;
}

void MonocularFeatureTrack::setTriangulatedPointWorld(
    const Eigen::Vector3d& point)
{
    triangulatedPointWorld_ = point;
}

void MonocularFeatureTrack::clearTriangulatedPointWorld()
{
    triangulatedPointWorld_.reset();
}

//AGE HANDLING
int MonocularFeatureTrack::getTrackAge() const noexcept
{
    return track_age;
}

void MonocularFeatureTrack::incrementTrackAge() noexcept
{
    ++track_age;
}

void MonocularFeatureTrack::resetTrackAge() noexcept
{
    track_age = 0;
}
