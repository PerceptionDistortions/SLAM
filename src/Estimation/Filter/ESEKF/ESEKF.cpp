#include "Estimation/Filter/ESEKF/ESEKF.h"

#include <Eigen/Core>
#include <stdexcept>
#include<iostream>

//CONSTRUCTOR
ESEKF::ESEKF(
    std::unique_ptr<Predictor> predictor,
    std::unique_ptr<Corrector> corrector,
    const FilterEstimatorConfig& config)
    : _predictor(std::move(predictor)),
      _corrector(std::move(corrector)),
      _config(config)
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

    //SET TO ZERO IN BEGIN OF EACH ITERATION
    //COMPUTED AGAIN IN THE CORRECTION STAGE
    _errorState.reset();

    //P COVARIANCE : INITIAL VALUE BASED ON CONFIGURATION PARAMETERS
    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();

    P.block<3, 3>(0, 0) = std::pow(_config.position_sd, 2) * Eigen::Matrix3d::Identity();
    P.block<3, 3>(3, 3) = std::pow(_config.velocity_sd, 2) * Eigen::Matrix3d::Identity();
    P.block<3, 3>(6, 6) = std::pow(_config.orientation_sd, 2) * Eigen::Matrix3d::Identity();
    P.block<3, 3>(9, 9) = std::pow(_config.acc_bias_sd, 2) * Eigen::Matrix3d::Identity();
    P.block<3, 3>(12, 12) = std::pow(_config.gyro_bias_sd, 2) * Eigen::Matrix3d::Identity();

    _covariance.set(P);
    _initialized = true;
}


void ESEKF::process(const Measurement& measurement)
{
    // std::cout << "ESEKF process: entered" << std::endl;

    //IS ESEKF INITIALIZED?
    if (!_initialized)
    {
        // std::cout << "ESEKF process: not initialized" << std::endl;
        throw std::runtime_error("ESEKF must be initialized before processing measurements.");
    }

    //COMPUTE TIME DIFFERENCE AND USE TIME GUARD
    const double dt =measurement.timestamp() -_nominalState.timestamp;
    // std::cout << "ESEKF process: dt=" << dt << std::endl;
    if (dt < 0.0) throw std::runtime_error("Measurement timestamp is older than estimator state.");

    // CHECK THE ROLE OF 
    const auto role = measurement.role();

    if (dt > 0.0 &&
        (role == MeasurementRole::Prediction ||
         role == MeasurementRole::PredictionAndCorrection))
    {
        // std::cout << "ESEKF process: before predict" << std::endl;
        predict(measurement);
        // std::cout << "ESEKF process: after predict" << std::endl;
    }

    if (role == MeasurementRole::Correction ||
        role == MeasurementRole::PredictionAndCorrection)
    {
        // std::cout << "ESEKF process: before correct" << std::endl;
        _corrector->correct(
            _nominalState,
            _errorState,
            _covariance,
            measurement);
        // std::cout << "ESEKF process: after correct" << std::endl;
    }

    // std::cout << "ESEKF process: before timestamp update" << std::endl;
    _nominalState.timestamp =measurement.timestamp();
    // std::cout << "ESEKF process: complete" << std::endl;
}

void ESEKF::predict(const Measurement& measurement)
{
    const Eigen::MatrixXd F = _predictor->computeF(_nominalState, measurement);
    const Eigen::MatrixXd Q = _predictor->computeQ(_nominalState, measurement);
    _predictor->predictNominal(_nominalState, measurement);
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

    estimate.timestamp =_nominalState.timestamp;
    estimate.position =_nominalState.position;
    estimate.velocity =_nominalState.velocity;
    estimate.orientation =_nominalState.orientation;

    return estimate;
}
