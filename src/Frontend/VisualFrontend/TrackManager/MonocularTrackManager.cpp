
#include "Frontend/VisualFrontend/TrackManager/MonocularTrackManager.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>
#include <iostream>

#include <algorithm>

//CONSTRUCTOR TAKES CONFIGURATION DATA ASSIGNS THEM
MonocularTrackManager::MonocularTrackManager(VisualTrackConfig& config)
    : tentative_max_misses(std::max(1, config.tentative_max_misses)),
      tentative_max_matches(std::max(1, config.tentative_max_match)),
      tentative_max_age(std::max(1, config.tentative_max_age)),

      active_max_misses(std::max(1, config.active_max_misses)),
      active_max_matches(std::max(1, config.active_max_matches)),
      active_max_age(std::max(1, config.active_max_age)),

      triangulated_max_observation(
          std::max(2, config.triangulated_max_observations)),
      triangulated_max_miss(
          std::max(1, config.triangulated_max_misses)),
      triangulated_max_matches(
          std::max(1, config.triangulated_max_matches)),

      triangulated_lost_max_miss(
          std::max(1, config.triangulated_lost_max_miss)),
      triangulated_lost_max_matches(
          std::max(1, config.triangulated_lost_max_match)),

      landmark_max_observation(
          std::max(2, config.landmark_max_observations)),
      landmark_max_miss(
          std::max(1, config.landmark_max_misses)),

      landmark_lost_max_miss(
          std::max(1, config.landmark_lost_max_misses)),
      landmark_lost_max_matches(
          std::max(1, config.landmark_lost_max_matches))
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
    if (!track.addObservation(observation)) return false;
    track.trimObservationHistory();
    return true;
}

//ASSOCIATE TO EXISTING TRACKS BASED ON MATCH STRENGTH
//CREATE NEW TENTATIVE TRACKS
//PROMOTE FROM TENTAIVE TO ACTIVE
//CAN DEMOTE AND DELETE ALL TYPES OF TRACKS IF NO MATCHES AND COUNT EXCEED
//DOES NOT PROMOTE OR DEMOTE TRIANGULATED AND LANDMARKS
void MonocularTrackManager::update(
    const std::vector<FeatureObservation>& currentObservations, //all feature obs
    const std::vector<cv::DMatch>& matches, //all success match between frames
    const std::vector<std::uint8_t>& geometricInlierMask)
{
    //COUNT OF TOTAL FEATURE OBSERVATIONS
    const std::size_t currentCount = currentObservations.size();

    //FOR ALL OBSERVATIONS, INIT SIZE OF TRACK IDs VECTOR
    currentFeatureTrackIds_.assign(currentCount, TrackId{0});

    //HOW MANY OLD TRACKS WILL BE CONTINUED (CAN BE LESS TOO)
    //WE RESERVE THE SIZE TO NUMBER OF MATCHES: MAX POSSIBLE
    //ADDED WHEN ASSOCIATED TO AN EXISTING TRACK
    std::unordered_set<TrackId> continuedTracks;
    continuedTracks.reserve(matches.size());

    //BOOLEAN: IS EACH FEATURE MAPPED TO A NEW OR EXISTING TRACK?: OBS COUNT
    std::vector<bool> currentAssigned(currentCount, false);

    // INDICES OF MATCHES THAT PASS GEOMETRIC VERIFICATION
    std::vector<std::size_t> candidateMatchIndices;
    candidateMatchIndices.reserve(matches.size());

    
    //TRACK ASSOCIATION CANDIDATES ONLY: NO NEW TRACK, NO STATE UPDATE, NO ASSIGNMENT
    //MULTIPLE COMMON MATCHES POSSIBLE
    //LOOP ON MATCHES: NOT FOR FIRST FRAME INIT
    for (std::size_t i = 0; i < matches.size(); ++i)
    {
        //GEOMETRIC MASK CHECK NOT POSSIBLE, SKIP THIS MATCH CHECK
        if (i >= geometricInlierMask.size() ||geometricInlierMask[i] == 0) continue;

        //CACHE THE MATCH
        const cv::DMatch& match = matches[i];
        
        //INVALID MATCHING QUERIES
        if (match.queryIdx < 0 || match.trainIdx < 0) continue;

        //INDICES OF MATCHED FEATURES IN TWO FRAMES KEYPOINTS
        const std::size_t previousIndex = static_cast<std::size_t>(match.queryIdx);
        const std::size_t currentIndex = static_cast<std::size_t>(match.trainIdx);

        //INDICES VALIDATION
        if (previousIndex >= previousFeatureTrackIds_.size() ||
            currentIndex >= currentCount) continue;

        //GET THE TRACK OF CURRENT FEATURE MATCH FROM PREV FRAME
        const TrackId trackId = previousFeatureTrackIds_[previousIndex];

        //IF NO TRACK ASSIGNED IN PREV FRAME, SKIP THE MATCH
        std::cout<<"Feature track missing in previous frame."<<std::endl;
        if (trackId == TrackId{0}) continue;
        
        candidateMatchIndices.push_back(i);
    }

    // SORT MATCHES STRONGEST TO WEAKEST, SKIP FOR FIRST FRAME
    // MAKE SURE SAME FEATURE DOES NOT MATCH TO MULTIPLE TRACKS/FEATURES
    std::sort(
        candidateMatchIndices.begin(),
        candidateMatchIndices.end(),
        [&matches](std::size_t lhs, std::size_t rhs)
        {
            return matches[lhs].distance < matches[rhs].distance;
        });

    //ASSOCIATE WITH EXISTING TRACKS FROM PREV FRAME: ONE TO ONE BEST MATCH
    //INCREMENT CONSECUTIVE COUNTER IN TRACK
    //PROMOTES A TENTAIVE TRACK ONLY
    for (const std::size_t matchIndex : candidateMatchIndices)
    {
        //CACHE THE CURRENT MATCH AND TAKE PREV AND CURR INDICES
        const cv::DMatch& match = matches[matchIndex];
        const std::size_t previousIndex =static_cast<std::size_t>(match.queryIdx);
        const std::size_t currentIndex =static_cast<std::size_t>(match.trainIdx);

        //ONE FEATURE WILL BE USED ONLY ONCE
        if (currentAssigned[currentIndex]) continue;
        
        //ONE TRACK WILL BE USED ONLY ONCE
        const TrackId trackId = previousFeatureTrackIds_[previousIndex]; //gte prev track from prev frame
        if (continuedTracks.find(trackId) != continuedTracks.end()) continue; //if already used skip it and skip frame
        MonocularFeatureTrack* track = getMonocularTrack(trackId); //get track object from the id
        if(track==nullptr) continue;

        if (!appendObservation(*track,currentObservations[currentIndex])) continue;
        track->incrementTrackAge();

        //IF FOUND, FAILURE COUNTER RESETS
        track->setConsecutiveInliers(track->getConsecutiveInliers() + 1);
        track->setConsecutiveFailures(0);

        // PROMOTE THE TENTATIVE TRACK
        if (track->getState() == TrackState::Tentative && track->getConsecutiveInliers() >=
                tentative_max_matches) track->setState(TrackState::Active);

        currentFeatureTrackIds_[currentIndex] = trackId;
        currentAssigned[currentIndex] = true;
        continuedTracks.insert(trackId);
    }

    // EXISTING TRACKS MISSED: UPDATE COUNTERS: CHECK IN MAP OF EXISTING TRACKS
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
        //CACHE ID AND TRACKS
        const TrackId trackId = it->first;
        MonocularFeatureTrack& track = *it->second;

        //IF MATCHES FOUND IN CURRENT THEN SKIP
        if (continuedTracks.find(trackId) != continuedTracks.end())
        {
            ++it;
            continue;
        }

        
        //TRACK WAS NOT FOUND AND SO THE COUNTER IS RESET
        //FAILURES INCREMENTED
        track.incrementTrackAge(); //increase age, relevant for tentative and active only
        track.setConsecutiveInliers(0);
        const int failures = track.getConsecutiveFailures() + 1;
        track.setConsecutiveFailures(failures);

        
        int maxMisses = 0;
        
        //DELETE THE TENTATIVE TRACK
        //DEMOTE THE ACTIVE TRACK
        //DELETE THE TRIANGULATED LOST TRACK
        //DEMOTE THE TRINAGULATE TO LOST
        //DELETE THE TRINAGULATED LOST
        //DEMOTE THE LANDMARK TO LOST
        //DELETE THE LANDMARK LOST, TRIGGER LANDMARK DELETION IN MAP MANAGER
        bool eraseTrack = false;

        switch (track.getState())
        {
            case TrackState::Tentative:
                eraseTrack = failures >= tentative_max_misses ||
                track.getTrackAge() >= tentative_max_age;
                break;

            case TrackState::Active:
                eraseTrack = failures >= active_max_misses ||
                track.getTrackAge() >= active_max_age;
                break;

            case TrackState::Triangulated:
                if (failures >= triangulated_max_miss)
                {
                    track.setState(TrackState::Triangulated_Lost);
                    track.setConsecutiveFailures(0);
                }
                break;

            case TrackState::Triangulated_Lost:
                eraseTrack = failures >= triangulated_lost_max_miss;
                break;

            case TrackState::Landmark:
                if (failures >= landmark_max_miss)
                {
                    track.setState(TrackState::Landmark_Lost);
                    track.setConsecutiveFailures(0);
                }
                break;

            case TrackState::Landmark_Lost:
                eraseTrack = failures >= landmark_lost_max_miss;
                break;
        }

        if (eraseTrack)
        {
            it = tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // NEW TENTATIVE TRACKS: CHECK WHICH ARE NOT ASSSIGNED
    for (std::size_t i = 0; i < currentCount; ++i)
    {
        if (currentAssigned[i]) continue;
        currentFeatureTrackIds_[i] = createTrack(currentObservations[i]);
    }

    // Preserve feature-index alignment for the next frame.
    previousFeatureTrackIds_ = currentFeatureTrackIds_;
    previousObservations_ = currentObservations;

    //REMOVE UNSTABLE TENTATIVE AND ACTIVE TRACKS
    prune();
}

bool MonocularTrackManager::markTriangulated(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() == TrackState::Triangulated_Lost ||
        track->getState() == TrackState::Landmark_Lost ||
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
        track->getState() == TrackState::Triangulated_Lost ||
        track->getState() == TrackState::Landmark_Lost ||
        track->getLandmarkId().has_value())
    {
        return false;
    }

    track->setLandmarkId(landmarkId);

    return true;
}


//TRACKS THAT HAV NOT BEEN PROMOTED OR DEMOTED OR STABLE FOR A LONG TIME
//APPLIES TO TENTATIVE AND ACTIVE TRACKS
// PRUNE ONLY TENTATIVE AND ACTIVE TRACKS THAT EXCEED THEIR AGE LIMIT
void MonocularTrackManager::prune()
{
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
        if (!it->second)
        {
            ++it;
            continue;
        }

        const TrackState state = it->second->getState();
        const int age = it->second->getTrackAge();

        const bool pruneTentative =
            state == TrackState::Tentative &&
            age >= tentative_max_age;

        const bool pruneActive =
            state == TrackState::Active &&
            age >= active_max_age;

        if (pruneTentative || pruneActive)
        {
            it = tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void MonocularTrackManager::reset()
{
    tracks_.clear();

    previousFeatureTrackIds_.clear();
    currentFeatureTrackIds_.clear();
    previousObservations_.clear();

    nextTrackId_ = 1;
}

//LOST TRACKS HANDLING
bool MonocularTrackManager::markTriangulatedLost(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() != TrackState::Triangulated)
    {
        return false;
    }

    track->setState(TrackState::Triangulated_Lost);
    track->setConsecutiveInliers(0);
    track->setConsecutiveFailures(0);

    return true;
}

bool MonocularTrackManager::recoverTriangulatedLost(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() != TrackState::Triangulated_Lost)
    {
        return false;
    }

    track->setState(TrackState::Triangulated);
    track->setConsecutiveInliers(0);
    track->setConsecutiveFailures(0);

    return true;
}

bool MonocularTrackManager::markLandmarkLost(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() != TrackState::Landmark)
    {
        return false;
    }

    track->setState(TrackState::Landmark_Lost);
    track->setConsecutiveInliers(0);
    track->setConsecutiveFailures(0);

    return true;
}

bool MonocularTrackManager::recoverLandmarkLost(TrackId trackId)
{
    MonocularFeatureTrack* track = getMonocularTrack(trackId);

    if (track == nullptr ||
        track->getState() != TrackState::Landmark_Lost)
    {
        return false;
    }

    track->setState(TrackState::Landmark);
    track->setConsecutiveInliers(0);
    track->setConsecutiveFailures(0);

    return true;
}
