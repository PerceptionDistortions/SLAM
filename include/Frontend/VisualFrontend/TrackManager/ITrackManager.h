
#pragma once

#include <vector>

#include "AliasesEnumsTypes/CommonAliases.h"
#include "Frontend/VisualFrontend/FeatureTracks/IFeatureTrack.h"

class ITrackManager
{
public:
    virtual ~ITrackManager() = default;

    //GET TRACK BY TRACK ID
    [[nodiscard]] virtual IFeatureTrack* getTrack(TrackId trackId) noexcept = 0;
    [[nodiscard]] virtual const IFeatureTrack* getTrack(TrackId trackId) const noexcept = 0;

    // GET LIST OF ALL TRACKS OF A PARTICULAR TYPE
    [[nodiscard]] virtual std::vector<TrackId> getTrackIds(TrackState state) const = 0;

    // ALL TRACKS
    [[nodiscard]] virtual std::vector<TrackId> getTrackIds() const = 0;

    //UPDATES TRACKS
    virtual void update(
    const std::vector<FeatureObservation>& currentObservations,
    const std::vector<cv::DMatch>& matches,
    const std::vector<std::uint8_t>& geometricInlierMask) = 0;

    //TRIANGULATED
    virtual bool markTriangulated(TrackId trackId) = 0;

    // Remove stale tracks and clear tracking state.
    virtual void prune() = 0;
    virtual void reset() = 0;
};
