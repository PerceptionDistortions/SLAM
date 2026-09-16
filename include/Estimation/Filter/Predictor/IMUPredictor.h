#pragma once

#include "Estimation/Filter/Predictor/Predictor.h"

class IMUPredictor : public Predictor
{
public:
    void predictNominal(
        NominalState& state,
        const Measurement& measurement) override;

    Eigen::MatrixXd computeF(
        const NominalState& state,
        const Measurement& measurement) const override;

    Eigen::MatrixXd computeQ(
        const NominalState& state,
        const Measurement& measurement) const override;
};