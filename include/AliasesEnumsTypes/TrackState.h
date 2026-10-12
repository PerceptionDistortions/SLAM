#pragma once

namespace slam{
enum class TrackState
{
    Tentative,
    Active,
    Lost,
    Triangulated,
    Landmark
};}