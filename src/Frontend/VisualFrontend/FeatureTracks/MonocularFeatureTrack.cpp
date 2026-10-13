
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
      lastFrameId_(initialObservation.frameId)
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

bool MonocularFeatureTrack::addObservation(
    const FeatureObservation& observation)
{
    // Do not accept duplicate or out-of-order frame observations.
    if (!observations_.empty() &&
        observation.frameId <= lastFrameId_)
    {
        return false;
    }

    observations_.push_back(observation);
    lastFrameId_ = observation.frameId;

    return true;
}

std::optional<LandmarkId>
MonocularFeatureTrack::getLandmarkId() const noexcept
{
    return landmarkId_;
}

void MonocularFeatureTrack::setLandmarkId(
    LandmarkId landmarkId)
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
    return lastFrameId_;
}

void MonocularFeatureTrack::trimObservationHistory(
    std::size_t maxObservations)
{
    if (maxObservations == 0 ||
        observations_.size() <= maxObservations)
    {
        return;
    }

    const std::size_t excess =
        observations_.size() - maxObservations;

    observations_.erase(
        observations_.begin(),
        observations_.begin() +
            static_cast<
                std::vector<FeatureObservation>::difference_type>(
                    excess));
}
