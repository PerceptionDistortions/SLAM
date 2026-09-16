#include "Estimation/Filter/ESEKF/ESEKF.h"

#include <Eigen/Core>
#include <stdexcept>

//CONSTRUCTOR
ESEKF::ESEKF(
    std::unique_ptr<Predictor> predictor,
    std::unique_ptr<Corrector> corrector)
    : _predictor(std::move(predictor)),
      _corrector(std::move(corrector))
{
    if (!_predictor) throw std::invalid_argument("ErrorStateEKF requires a predictor.");
    if (!_corrector) throw std::invalid_argument("ErrorStateEKF requires a corrector.");
}

void ESEKF::initialize(const StateEstimate& initialState)
{
    _nominalState.position = initialState.position;
    _nominalState.velocity = initialState.velocity;
    _nominalState.orientation = initialState.orientation.normalized();
    _nominalState.timestamp = initialState.timestamp;
    _errorState.reset();

    /*
        Initial covariance.

        These values should eventually be configurable.
    */

    Eigen::Matrix<double, 15, 15> P;
    P.setZero();

    P.block<3, 3>(0, 0) = 0.1 * 0.1 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(3, 3) = 0.5 * 0.5 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(6, 6) = 0.1 * 0.1 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(9, 9) = 0.01 * 0.01 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(12, 12) = 0.01 * 0.01 * Eigen::Matrix3d::Identity();

    _covariance.set(P);
    _initialized = true;
}


void ESEKF::process(const Measurement& measurement)
{
    //INIT CHECK
    if (!_initialized)
    {
        throw std::runtime_error("ESEKF must be initialized before processing measurements.");
    }

    const double dt =measurement.timestamp() -_nominalState.timestamp;

    //TIME GUARD
    if (dt < 0.0)
    {
        throw std::runtime_error("Measurement timestamp is older than estimator state.");
    }

    const auto role = measurement.role();

    if (dt > 0.0 &&
        (role == MeasurementRole::Prediction ||
         role == MeasurementRole::PredictionAndCorrection))
    {
        predict(measurement);
    }

    if (role == MeasurementRole::Correction ||
        role == MeasurementRole::PredictionAndCorrection)
    {
        _corrector->correct(
            _nominalState,
            _errorState,
            _covariance,
            measurement);
    }

    _nominalState.timestamp =
        measurement.timestamp();
}

void ESEKF::predict(
    const Measurement& measurement)
{
    // 1. Propagate nominal state
    _predictor->predictNominal(_nominalState,measurement);

    // 2. Get model-specific error-state Jacobian
    const Eigen::MatrixXd F =
        _predictor->computeF(
            _nominalState,
            measurement);

    // 3. Get model-specific process noise
    const Eigen::MatrixXd Q =
        _predictor->computeQ(
            _nominalState,
            measurement);

    // 4. Propagate covariance
    propagateCovariance(F, Q);
}


void ESEKF::propagateCovariance(
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& Q)
{
    const Eigen::MatrixXd& P =
        _covariance.matrix();

    const Eigen::MatrixXd propagatedP =
        F * P * F.transpose() + Q;

    _covariance.matrix() = propagatedP;
}


StateEstimate ESEKF::getEstimate() const
{
    StateEstimate estimate;

    estimate.timestamp =
        _nominalState.timestamp;

    estimate.position =
        _nominalState.position;

    estimate.velocity =
        _nominalState.velocity;

    estimate.orientation =
        _nominalState.orientation;

    return estimate;
}