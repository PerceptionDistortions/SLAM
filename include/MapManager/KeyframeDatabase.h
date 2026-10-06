#pragma once

#include <memory>
#include <vector>

#include "MapManager/KeyframesLandmarks/Keyframe.h"

class KeyframeDatabase
{
public:
    KeyframeDatabase() = default;
    ~KeyframeDatabase() = default;

    void addKeyframe(
        const std::shared_ptr<Keyframe>& keyframe);

    void removeKeyframe(
        const std::shared_ptr<Keyframe>& keyframe);

    std::vector<std::shared_ptr<Keyframe>>
    queryCandidates(
        const std::shared_ptr<Keyframe>& keyframe) const;

private:
    std::vector<std::shared_ptr<Keyframe>> keyframes_;
};