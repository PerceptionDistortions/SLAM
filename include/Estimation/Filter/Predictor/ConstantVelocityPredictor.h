#pragma once

#include "Estimation/Filter/Predictor/Predictor.h"
#include "Configuration/NewConfigs/FilterEstimatorConfig.h"

class ConstantVelocityPredictor : public Predictor
{
public:
    explicit ConstantVelocityPredictor(const FilterEstimatorConfig& config);

    void predictNominal(
        NominalState& state,
        const Measurement& measurement) override;

    Eigen::MatrixXd computeF(
        const NominalState& state,
        const Measurement& measurement) const override;

    Eigen::MatrixXd computeQ(
        const NominalState& state,
        const Measurement& measurement) const override;

private:
    FilterEstimatorConfig _config;
};