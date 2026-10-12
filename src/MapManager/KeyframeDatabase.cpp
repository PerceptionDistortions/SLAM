#include "MapManager/KeyframeDatabase.h"

#include <algorithm>

void KeyframeDatabase::addKeyframe(
    const std::shared_ptr<Keyframe>& keyframe)
{
    if (!keyframe)
        return;

    const auto it = std::find_if(
        keyframes_.begin(),
        keyframes_.end(),
        [&keyframe](const std::shared_ptr<Keyframe>& existing)
        {
            return existing &&
                   existing->getId() == keyframe->getId();
        });

    if (it == keyframes_.end())
        keyframes_.push_back(keyframe);
}

void KeyframeDatabase::removeKeyframe(
    const std::shared_ptr<Keyframe>& keyframe)
{
    if (!keyframe)
        return;

    const KeyframeId id = keyframe->getId();

    keyframes_.erase(
        std::remove_if(
            keyframes_.begin(),
            keyframes_.end(),
            [id](const std::shared_ptr<Keyframe>& existing)
            {
                return existing && existing->getId() == id;
            }),
        keyframes_.end());
}