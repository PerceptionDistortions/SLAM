#pragma once

#include"Backend/BA/BundleAdjustment.h"

class LocalBundleAdjustment : public BundleAdjustment
{
public:
    bool optimize(MapManager& map) override;
};