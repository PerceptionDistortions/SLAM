#pragma once

enum class TrackState
{
    Tentative,
    Active,
    Triangulated,
    Landmark,
    Triangulated_Lost,
    Landmark_Lost
};