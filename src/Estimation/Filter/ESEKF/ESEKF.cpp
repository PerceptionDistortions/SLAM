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

    Eigen::Matrix<double, 9, 9> P;
    P.setZero();

    P.block<3, 3>(0, 0) = 0.1 * 0.1 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(3, 3) = 0.5 * 0.5 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(6, 6) = 0.1 * 0.1 * Eigen::Matrix3d::Identity();
    _covariance.set(P);
    _initialized = true;
}


void ESEKF::process(
    const Measurement& measurement)
{
    if (!_initialized)
    {
        throw std::runtime_error(
            "ErrorStateEKF must be initialized before processing measurements.");
    }

    const double timestamp = measurement.timestamp();
    const double dt = timestamp - _nominalState.timestamp;
    if (dt < 0.0)
    {
        throw std::runtime_error(
            "Measurement timestamp is older than estimator state.");
    }

    /*
        Prediction
    */

    if (dt > 0.0)
    {
        predict(dt);
    }

    /*
        Correction
    */

    _corrector->correct(
        _nominalState,
        _errorState,
        _covariance,
        measurement);

    _nominalState.timestamp = timestamp;
}


void ESEKF::predict(double dt)
{
    /*
        Propagate nominal state.
    */

    _predictor->predict(
        _nominalState,
        dt);

    /*
        Propagate covariance.
    */

    propagateCovariance(dt);
}


void ESEKF::propagateCovariance(double dt)
{
    /*
        Constant-velocity error-state transition:

        δp' = δp + δv dt
        δv' = δv
        δθ' = δθ

        Therefore:

        F =

        [ I  I*dt  0
          0   I    0
          0   0    I ]
    */

    Eigen::Matrix<double, 9, 9> F =
        Eigen::Matrix<double, 9, 9>::Identity();

    F.block<3, 3>(0, 3) =
        dt * Eigen::Matrix3d::Identity();

    /*
        Process noise.

        This is intentionally simple.

        A more complete implementation would derive Q
        from acceleration/angular-velocity noise.
    */

    Eigen::Matrix<double, 9, 9> Q;
    Q.setZero();

    constexpr double positionNoise = 0.01;
    constexpr double velocityNoise = 0.05;
    constexpr double orientationNoise = 0.01;

    Q.block<3, 3>(0, 0) =
        positionNoise * positionNoise *
        dt * Eigen::Matrix3d::Identity();

    Q.block<3, 3>(3, 3) =
        velocityNoise * velocityNoise *
        dt * Eigen::Matrix3d::Identity();

    Q.block<3, 3>(6, 6) =
        orientationNoise * orientationNoise *
        dt * Eigen::Matrix3d::Identity();

    const Eigen::Matrix<double, 9, 9>& P =
        _covariance.matrix();

    Eigen::Matrix<double, 9, 9> propagatedP =
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