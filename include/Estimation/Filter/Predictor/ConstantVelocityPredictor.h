#pragma once

#include "Filter/Predictor/Predictor.h"

class ConstantVelocityPredictor : public Predictor
{
public:
    void predict(
        NominalState& state,
        double dt) override;
};