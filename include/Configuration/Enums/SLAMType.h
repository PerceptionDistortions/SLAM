#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

enum class SLAMType
{
    Monocular,
    MonocularVI,
    StereoVI
};

//RETURNS THE SLAM TYPE FROM CONFIG STRING USING ENUM
inline SLAMType slamTypeFromString(std::string_view value)
{
    if (value == "monocular")
        return SLAMType::Monocular;

    if (value == "monocular_vi")
        return SLAMType::MonocularVI;

    if (value == "stereo_vi")
        return SLAMType::StereoVI;

    throw std::invalid_argument(
        "Invalid SLAM mode: " + std::string(value));
}