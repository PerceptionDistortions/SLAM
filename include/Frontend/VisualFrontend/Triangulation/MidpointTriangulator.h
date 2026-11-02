#pragma once

#include "Frontend/VisualFrontend/Triangulation/ITriangulator.h"
#include"Configuration/NewConfigs/TriangulationConfig.h"

class MidpointTriangulator final : public ITriangulator
{
public:
    explicit MidpointTriangulator(TriangulationConfig config = {});

    [[nodiscard]]
    TriangulationResult triangulate(
        const TriangulationObservation& first,
        const TriangulationObservation& second) const override;

private:
    TriangulationConfig config_;
};