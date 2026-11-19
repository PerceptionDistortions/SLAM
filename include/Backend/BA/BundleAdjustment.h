#pragma once

#include"MapManager/MapManager.h"

class BundleAdjustment
{
public:
    virtual ~BundleAdjustment() = default;
    virtual bool optimize(MapManager& map) = 0;
};