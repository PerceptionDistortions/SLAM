#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"

#include <Eigen/Core>

ConstantVelocityPredictor::ConstantVelocityPredictor(const FilterEstimatorConfig& config)
    : _config(config)
{
}

void ConstantVelocityPredictor::predictNominal(
    NominalState& state,
    const Measurement& measurement)
{
    const double dt = measurement.timestamp() - state.timestamp;
    state.position += state.velocity * dt;

    // Velocity remains constant.
    // Orientation remains constant.
    // IMU biases remain constant.

    state.normalizeOrientation();
}

Eigen::MatrixXd ConstantVelocityPredictor::computeF(
    const NominalState& state,
    const Measurement& measurement) const
{
    const double dt = measurement.timestamp() - state.timestamp;
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
    F.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
    return F;
}

Eigen::MatrixXd ConstantVelocityPredictor::computeQ(
    const NominalState& state,
    const Measurement& measurement) const
{
    const double dt = measurement.timestamp() - state.timestamp;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    const double accelerationNoise = std::pow(_config.acceleration_noise_sd, 2);

    Q.block<3, 3>(0, 0) = (dt3 / 3.0) * accelerationNoise * I;
    Q.block<3, 3>(0, 3) = (dt2 / 2.0) * accelerationNoise * I;
    Q.block<3, 3>(3, 0) = (dt2 / 2.0) * accelerationNoise * I;
    Q.block<3, 3>(3, 3) = dt * accelerationNoise * I;

    Q.block<3, 3>(6, 6) = std::pow(_config.orientation_noise_sd, 2) * dt * I;
    Q.block<3, 3>(9, 9) = std::pow(_config.acc_bias_noise_sd, 2) * dt * I;
    Q.block<3, 3>(12, 12) = std::pow(_config.gyro_bias_noise_sd, 2) * dt * I;

    return Q;
}