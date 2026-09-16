#pragma once

#include "Estimation/Filter/State/NominalState.h"
#include "Estimation/Measurement/Measurement.h"

#include <Eigen/Core>

class Predictor
{
public:
    virtual ~Predictor() = default;

    virtual void predictNominal(
        NominalState& state,
        const Measurement& measurement) = 0;

    // Error-state transition Jacobian
    virtual Eigen::MatrixXd computeF(
        const NominalState& state,
        const Measurement& measurement) const = 0;

    // Process-noise covariance
    virtual Eigen::MatrixXd computeQ(
        const NominalState& state,
        const Measurement& measurement) const = 0;
};