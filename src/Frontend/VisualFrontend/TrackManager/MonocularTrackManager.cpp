
#include "Frontend/VisualFrontend/TrackManager/MonocularTrackManager.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>
#include<iostream>

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
    //PROMOTE COUNT TENTATIVE->ACTIVE
    //ADD THIS IN HEADER AND EXTRACT FROM CONFIGURATION LATER
    constexpr int kConsecutiveInliersToActivate = 2;

    //COUNT OF TOTAL FEATURE OBSERVATIONS
    const std::size_t currentCount = currentObservations.size();

    //VECTOR OF ALL TRACKS IN CURRENT FRAME
    //DOES NOT ASSIGN ID: INIT TO 0, ASSIGN LATER
    currentFeatureTrackIds_.assign(currentCount, TrackId{0});

    //HOW MANY OLD TRACKS WILL BE CONTINUED
    //WE RESERVE THE SIZE TO NUMBER OF MATCHES: MAX POSSIBLE
    std::unordered_set<TrackId> continuedTracks;
    continuedTracks.reserve(matches.size());

    //BOOLEAN: IS EACH FEATURE MAPPED TO A NEW OR EXISTING TRACK?
    //COUNT=OBSERVATION COUNT
    std::vector<bool> currentAssigned(currentCount, false);

    // INDICES OF MATCHES THAT PASS GEOMETRIC VERIFICATION
    std::vector<std::size_t> candidateMatchIndices;
    candidateMatchIndices.reserve(matches.size());

    //RUN LOOP ON ALL THE MATCHES: SKIPPED FOR FIRST FRAME AS NO MATCHES
    //MATCHED FEATURES: PREV FEATURE IF HAS TRACK NEW ONE TOO HAS TRACK
    //ADD IF ASSOCIATED WITH AN EXISTING TRACK
    //NO NEW TRACK YET
    //NO STATE UPDATE
    for (std::size_t i = 0; i < matches.size(); ++i)
    {
        //GEOMETRIC MASK CHECK NOT POSSIBLE, SKIP THIS MATCH CHECK
        if (i >= geometricInlierMask.size() ||geometricInlierMask[i] == 0) continue;

        //CACHE THE MATCH
        const cv::DMatch& match = matches[i];
        
        if (match.queryIdx < 0 || match.trainIdx < 0) continue;

        //INDICES OF MATCHED FEATURES IN TWO FRAMES KEYPOINTS
        const std::size_t previousIndex = static_cast<std::size_t>(match.queryIdx);
        const std::size_t currentIndex = static_cast<std::size_t>(match.trainIdx);

        //CHECK OF VALID INDICES
        if (previousIndex >= previousFeatureTrackIds_.size() ||
            currentIndex >= currentCount) continue;

        //GET THE TRACK OF CURRENT FEATURE MATCH FROM PREV FRAME
        const TrackId trackId = previousFeatureTrackIds_[previousIndex];

        //IF NO TRACK ASSIGNED IN PREV FRAME, SKIP THE MATCH
        std::cout<<"Feature track missing in previous frame."<<std::endl;
        if (trackId == TrackId{0}) continue;
        
        candidateMatchIndices.push_back(i);
    }

    // SORT ALL THE MATCHES: STRONGEST TO WEAKEST
    // SKIPPED FOR THE FIRST FRAME INIT
    // MAKE SURE SAME FEATURE DOES NOT MATCH TO MULTIPLE TRACKS/FEATURES
    std::sort(
        candidateMatchIndices.begin(),
        candidateMatchIndices.end(),
        [&matches](std::size_t lhs, std::size_t rhs)
        {
            return matches[lhs].distance < matches[rhs].distance;
        });

    // Assign verified matches one-to-one:
    // - each current feature receives at most one track;
    // - each existing track continues at most once per frame.
    for (const std::size_t matchIndex : candidateMatchIndices)
    {
        //CACHE THE CURRENT MATCH AND TAKE PREV AND CURR INDICES
        const cv::DMatch& match = matches[matchIndex];
        const std::size_t previousIndex =static_cast<std::size_t>(match.queryIdx);
        const std::size_t currentIndex =static_cast<std::size_t>(match.trainIdx);

        //BOOLEAN DEFINE BEFORE: IS ASSIGNED TO A TRACK?
        //IF ALREADY ASSIGNED SKIP
        if (currentAssigned[currentIndex]) continue;
        
        //PREVIOUS FRAME TRACKS ARE ALREADY STORED, GET TRACK ID FOR CURR TRACK
        //CONVERT TO MONOCULAR TRACK
        const TrackId trackId = previousFeatureTrackIds_[previousIndex];
        if (continuedTracks.find(trackId) != continuedTracks.end()) continue;
        MonocularFeatureTrack* track = getMonocularTrack(trackId);
        if(track==nullptr) continue;

        // Preserve existing Lost-state behavior.
        // if (track == nullptr ||track->getState() == TrackState::Lost)
        // {
        //     continue;
        // }

        if (!appendObservation(
                *track,
                currentObservations[currentIndex]))
        {
            continue;
        }

        track->setConsecutiveInliers(
            track->getConsecutiveInliers() + 1);

        track->setConsecutiveFailures(0);

        // Promote after two consecutive successful continuations.
        if (track->getState() == TrackState::Tentative &&
            track->getConsecutiveInliers() >=
                kConsecutiveInliersToActivate)
        {
            track->setState(TrackState::Active);
        }

        currentFeatureTrackIds_[currentIndex] = trackId;
        currentAssigned[currentIndex] = true;
        continuedTracks.insert(trackId);
    }

    // Update misses and preserve the existing deletion policy.
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

        const int failures =
            track.getConsecutiveFailures() + 1;

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

    // Create tentative tracks for unassigned current features.
    for (std::size_t i = 0; i < currentCount; ++i)
    {
        if (currentAssigned[i])
        {
            continue;
        }

        currentFeatureTrackIds_[i] =
            createTrack(currentObservations[i]);
    }

    // Preserve feature-index alignment for the next frame.
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
