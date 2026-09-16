#pragma once

#include "Estimation/Filter/State/NominalState.h"
#include "Estimation/Filter/State/ErrorState.h"
#include "Estimation/Filter/Uncertainty/Uncertainty.h"
#include "Estimation/Measurement/Measurement.h"

class Corrector
{
public:
    virtual ~Corrector() = default;

    virtual void correct(
        NominalState& nominalState,
        ErrorState& errorState,
        Uncertainty& uncertainty,
        const Measurement& measurement) = 0;
};