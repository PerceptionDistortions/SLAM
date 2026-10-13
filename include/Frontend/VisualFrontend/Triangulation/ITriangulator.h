#pragma once

#include"Frontend/VisualFrontend/Triangulation/TriangulationFailure.h"

class ITriangulator
{
public:
    virtual ~ITriangulator() = default;

    [[nodiscard]]
    virtual TriangulationResult triangulate(
        const TriangulationObservation& first,
        const TriangulationObservation& second) const = 0;
};