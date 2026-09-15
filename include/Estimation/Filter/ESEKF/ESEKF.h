#pragma once

#include "Filter/FilterEstimator.h"

#include "Filter/State/NominalState.h"
#include "Filter/State/ErrorState.h"

#include "Filter/Uncertainty/Covariance.h"

#include "Filter/Predictor/Predictor.h"
#include "Filter/Corrector/Corrector.h"

#include <memory>

class ESEKF : public FilterEstimator
{
public:
    ErrorStateEKF(
        std::unique_ptr<Predictor> predictor,
        std::unique_ptr<Corrector> corrector);

    ~ESEKF() override = default;

    void initialize(const StateEstimate& initialState) override;

    //THIS IS PART OF ESTIMATOR ABSTRACTION
    //DONE DIFF THEN OPTIMIZER
    void process(const Measurement& measurement) override;
    StateEstimate getEstimate() const override;

private:
    void predict(double dt);

    void propagateCovariance(double dt);

    void injectStateEstimate();

private:
    NominalState _nominalState;
    ErrorState _errorState;
    Covariance _covariance;

    std::unique_ptr<Predictor> _predictor;
    std::unique_ptr<Corrector> _corrector;

    bool _initialized{false};
};