
#include "Frontend/VisualFrontend/TrackManager/MonocularTrackManager.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

MonocularTrackManager::MonocularTrackManager(
    int maxConsecutiveFailures,
    std::size_t maxObservationHistory)
    : maxConsecutiveFailures_(
          std::max(1, maxConsecutiveFailures)),
      maxObservationHistory_(
          std::max<std::size_t>(2, maxObservationHistory))
{
}

IFeatureTrack* MonocularTrackManager::getTrack(
    TrackId trackId) noexcept
{
    const auto it = tracks_.find(trackId);

    return it != tracks_.end() ? it->second.get() : nullptr;
}

const IFeatureTrack* MonocularTrackManager::getTrack(
    TrackId trackId) const noexcept
{
    const auto it = tracks_.find(trackId);

    return it != tracks_.end() ? it->second.get() : nullptr;
}

MonocularFeatureTrack*
MonocularTrackManager::getMonocularTrack(
    TrackId trackId) noexcept
{
    const auto it = tracks_.find(trackId);

    return it != tracks_.end() ? it->second.get() : nullptr;
}

const MonocularFeatureTrack*
MonocularTrackManager::getMonocularTrack(
    TrackId trackId) const noexcept
{
    const auto it = tracks_.find(trackId);

    return it != tracks_.end() ? it->second.get() : nullptr;
}

std::vector<TrackId> MonocularTrackManager::getTrackIds(
    TrackState state) const
{
    std::vector<TrackId> result;
    result.reserve(tracks_.size());

    for (const auto& [trackId, track] : tracks_)
    {
        if (track && track->getState() == state)
        {
            result.push_back(trackId);
        }
    }

    return result;
}

std::vector<TrackId> MonocularTrackManager::getTrackIds() const
{
    std::vector<TrackId> result;
    result.reserve(tracks_.size());

    for (const auto& [trackId, track] : tracks_)
    {
        if (track)
        {
            result.push_back(trackId);
        }
    }

    return result;
}

TrackId MonocularTrackManager::createTrack(const FeatureObservation& observation)
{
    const TrackId trackId = nextTrackId_++;
    auto track = std::make_unique<MonocularFeatureTrack>(trackId,observation);
    tracks_.emplace(trackId, std::move(track));
    return trackId;
}

bool MonocularTrackManager::appendObservation(MonocularFeatureTrack& track,
    const FeatureObservation& observation)
{
    if (track.getState() == TrackState::Lost) return false;
    if (!track.addObservation(observation)) return false;
    track.trimObservationHistory(maxObservationHistory_);
    return true;
}

void MonocularTrackManager::update(
    const std::vector<FeatureObservation>& currentObservations,
    const std::vector<cv::DMatch>& matches,
    const std::vector<std::uint8_t>& geometricInlierMask)
{
    const std::size_t currentCount = currentObservations.size();

    currentFeatureTrackIds_.assign(currentCount, TrackId{0});

    std::unordered_set<TrackId> continuedTracks;
    continuedTracks.reserve(matches.size());

    std::vector<bool> currentAssigned(currentCount, false);

    // If multiple previous features match one current feature, keep
    // the geometrically verified match with the lowest descriptor distance.
    std::vector<std::size_t> bestMatchForCurrent(
        currentCount, matches.size());

    for (std::size_t i = 0; i < matches.size(); ++i)
    {
        if (i >= geometricInlierMask.size() ||
            geometricInlierMask[i] == 0)
        {
            continue;
        }

        const cv::DMatch& match = matches[i];
        if (match.queryIdx < 0 || match.trainIdx < 0)
        {
            continue;
        }

        const std::size_t previousIndex =
            static_cast<std::size_t>(match.queryIdx);
        const std::size_t currentIndex =
            static_cast<std::size_t>(match.trainIdx);

        if (previousIndex >= previousFeatureTrackIds_.size() ||
            currentIndex >= currentCount ||
            previousFeatureTrackIds_[previousIndex] == TrackId{0})
        {
            continue;
        }

        const std::size_t bestIndex = bestMatchForCurrent[currentIndex];
        if (bestIndex == matches.size() ||
            match.distance < matches[bestIndex].distance)
        {
            bestMatchForCurrent[currentIndex] = i;
        }
    }

    // Continue tracks using geometrically verified matches.
    for (std::size_t i = 0; i < matches.size(); ++i)
    {
        // Mask entries must correspond to matches[i].
        if (i >= geometricInlierMask.size() ||
            geometricInlierMask[i] == 0)
        {
            continue;
        }

        const cv::DMatch& match = matches[i];

        if (match.queryIdx < 0 || match.trainIdx < 0)
        {
            continue;
        }

        const std::size_t previousIndex =static_cast<std::size_t>(match.queryIdx);
        const std::size_t currentIndex =static_cast<std::size_t>(match.trainIdx);

        if (previousIndex >= previousFeatureTrackIds_.size() ||currentIndex >= currentCount) continue;
        if (bestMatchForCurrent[currentIndex] != i) continue;
        if (currentAssigned[currentIndex]) continue;
        const TrackId trackId = previousFeatureTrackIds_[previousIndex];
        if (trackId == TrackId{0}) continue;

        // One track may continue at most once per frame.
        if (continuedTracks.find(trackId) != continuedTracks.end())
        {
            continue;
        }

        MonocularFeatureTrack* track = getMonocularTrack(trackId);

        if (track == nullptr || track->getState() == TrackState::Lost) continue;

        if (!appendObservation(*track,currentObservations[currentIndex]))
        {
            continue;
        }

        track->setConsecutiveInliers(track->getConsecutiveInliers() + 1);
        track->setConsecutiveFailures(0);

        // Promote a tentative track after its first successful
        // geometrically verified continuation.
        if (track->getState() == TrackState::Tentative && track->getConsecutiveInliers() >= 3)
        {
            track->setState(TrackState::Active);
        }

        currentFeatureTrackIds_[currentIndex] = trackId;
        currentAssigned[currentIndex] = true;
        continuedTracks.insert(trackId);
    }

    
    // Count misses for every existing track not continued this frame.
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
        const TrackId trackId = it->first;
        MonocularFeatureTrack& track = *it->second;

        if (continuedTracks.find(trackId) != continuedTracks.end())
        {
            ++it;
            continue;
        }

        track.setConsecutiveInliers(0);

        const int failures = track.getConsecutiveFailures() + 1;
        track.setConsecutiveFailures(failures);

        if (failures >= maxConsecutiveFailures_)
        {
            it = tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }


    // Unassigned current observations start new tentative tracks.
    for (std::size_t i = 0; i < currentCount; ++i)
    {
        if (currentAssigned[i])
        {
            continue;
        }

        currentFeatureTrackIds_[i] =
            createTrack(currentObservations[i]);
    }

    // Preserve exact feature-index alignment for the next frame.
    previousFeatureTrackIds_ = currentFeatureTrackIds_;
    previousObservations_ = currentObservations;
}

bool MonocularTrackManager::markTriangulated(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() == TrackState::Lost ||
        track->getObservations().size() < 2)
    {
        return false;
    }

    track->setState(TrackState::Triangulated);

    return true;
}

bool MonocularTrackManager::associateLandmark(
    TrackId trackId,
    LandmarkId landmarkId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() == TrackState::Lost ||
        track->getLandmarkId().has_value())
    {
        return false;
    }

    track->setLandmarkId(landmarkId);

    return true;
}

bool MonocularTrackManager::markLost(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr)
    {
        return false;
    }

    track->setState(TrackState::Lost);

    return true;
}

void MonocularTrackManager::prune()
{
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
        if (it->second &&
            it->second->getState() == TrackState::Lost)
        {
            it = tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Keep the index-aligned vectors unchanged. Any stale IDs
    // are ignored by getTrack() when processing the next frame.
}

void MonocularTrackManager::reset()
{
    tracks_.clear();

    previousFeatureTrackIds_.clear();
    currentFeatureTrackIds_.clear();
    previousObservations_.clear();

    nextTrackId_ = 1;
}
