#pragma once

#include "StateEstimate.h"
#include "Measurement.h"

class Estimator
{
public:
    virtual ~Estimator() = default;

    virtual void initialize(const StateEstimate& initialState) = 0;

    virtual void process(const Measurement& measurement) = 0;

    virtual StateEstimate getEstimate() const = 0;
};