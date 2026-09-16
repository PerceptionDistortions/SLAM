#pragma once

#include "Estimation/Filter/FilterEstimator.h"

#include "Estimation/Filter/State/NominalState.h"
#include"Estimation/Filter/State/ErrorState.h"

#include "Estimation/Filter/Uncertainty/Covariance.h"

#include "Estimation/Filter/Predictor/Predictor.h"
#include "Estimation/Filter/Corrector/Corrector.h"

#include <memory>
#include <Eigen/Core>

class ESEKF : public FilterEstimator
{
public:
    ESEKF(
        std::unique_ptr<Predictor> predictor,
        std::unique_ptr<Corrector> corrector);

    ~ESEKF() override = default;

    void initialize(
        const StateEstimate& initialState) override;

    void process(
        const Measurement& measurement) override;

    StateEstimate getEstimate() const override;

private:
    void predict(
        const Measurement& measurement);

    void propagateCovariance(
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& Q);

    void injectStateEstimate();

private:
    NominalState _nominalState;
    ErrorState _errorState;
    Covariance _covariance;

    std::unique_ptr<Predictor> _predictor;
    std::unique_ptr<Corrector> _corrector;

    bool _initialized{false};
};