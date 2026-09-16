#pragma once

#include "Estimation/StateEstimator.h"
#include "Estimation/Measurement/Measurement.h"

class Estimator
{
public:
    virtual ~Estimator() = default;

    virtual void initialize(
        const StateEstimate& initialState) = 0;

    virtual void process(
        const Measurement& measurement) = 0;

    virtual StateEstimate getEstimate() const = 0;
};