#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"

#include <Eigen/Core>

void ConstantVelocityPredictor::predictNominal(
    NominalState& state,
    const Measurement& measurement)
{
    const double dt =
        measurement.timestamp() - state.timestamp;

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
    const double dt =
        measurement.timestamp() - state.timestamp;

    Eigen::Matrix<double, 15, 15> F =
        Eigen::Matrix<double, 15, 15>::Identity();

    F.block<3, 3>(0, 3) =
        dt * Eigen::Matrix3d::Identity();

    return F;
}

Eigen::MatrixXd ConstantVelocityPredictor::computeQ(
    const NominalState& state,
    const Measurement& measurement) const
{
    const double dt =
        measurement.timestamp() - state.timestamp;

    Eigen::Matrix<double, 15, 15> Q;
    Q.setZero();

    constexpr double positionNoise = 0.01;
    constexpr double velocityNoise = 0.05;
    constexpr double orientationNoise = 0.01;
    constexpr double accelerometerBiasNoise = 0.001;
    constexpr double gyroscopeBiasNoise = 0.001;

    Q.block<3, 3>(0, 0) =
        positionNoise * positionNoise *
        dt * Eigen::Matrix3d::Identity();

    Q.block<3, 3>(3, 3) =
        velocityNoise * velocityNoise *
        dt * Eigen::Matrix3d::Identity();

    Q.block<3, 3>(6, 6) =
        orientationNoise * orientationNoise *
        dt * Eigen::Matrix3d::Identity();

    Q.block<3, 3>(9, 9) =
        accelerometerBiasNoise *
        accelerometerBiasNoise *
        dt *
        Eigen::Matrix3d::Identity();

    Q.block<3, 3>(12, 12) =
        gyroscopeBiasNoise *
        gyroscopeBiasNoise *
        dt *
        Eigen::Matrix3d::Identity();

    return Q;
}