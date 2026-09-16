#include "Estimation/Filter/Corrector/MonocularVisualCorrector.h"

#include "Estimation/Measurement/MonocularVisualMeasurement.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdexcept>

namespace
{
    constexpr int ErrorStateDimension = 15;
    constexpr int MeasurementDimension = 6;
}

Eigen::Vector3d
MonocularVisualCorrector::orientationError(
    const Eigen::Quaterniond& measured,
    const Eigen::Quaterniond& estimated)
{
    Eigen::Quaterniond delta =
        measured * estimated.conjugate();

    delta.normalize();

    if (delta.w() < 0.0)
    {
        delta.coeffs() *= -1.0;
    }

    return 2.0 * delta.vec();
}

Eigen::Quaterniond
MonocularVisualCorrector::applyOrientationError(
    const Eigen::Vector3d& deltaTheta,
    const Eigen::Quaterniond& orientation)
{
    const double angle =
        deltaTheta.norm();

    Eigen::Quaterniond deltaQuaternion;

    if (angle < 1e-12)
    {
        deltaQuaternion =
            Eigen::Quaterniond::Identity();
    }
    else
    {
        deltaQuaternion =
            Eigen::Quaterniond(
                Eigen::AngleAxisd(
                    angle,
                    deltaTheta / angle));
    }

    Eigen::Quaterniond updated =
        deltaQuaternion * orientation;

    updated.normalize();

    return updated;
}

void MonocularVisualCorrector::correct(
    NominalState& nominalState,
    ErrorState& errorState,
    Uncertainty& uncertainty,
    const Measurement& measurement)
{
    const auto* visualMeasurement =
        dynamic_cast<const MonocularVisualMeasurement*>(
            &measurement);

    if (!visualMeasurement)
    {
        throw std::invalid_argument(
            "MonocularVisualCorrector received "
            "unsupported measurement type.");
    }

    Eigen::Matrix<
        double,
        MeasurementDimension,
        ErrorStateDimension> H;

    H.setZero();

    // Position measurement observes position error.
    H.block<3, 3>(0, 0).setIdentity();

    // Orientation measurement observes orientation error.
    H.block<3, 3>(3, 6).setIdentity();

    Eigen::Matrix<
        double,
        MeasurementDimension,
        1> residual;

    residual.segment<3>(0) =
        visualMeasurement->position() -
        nominalState.position;

    residual.segment<3>(3) =
        orientationError(
            visualMeasurement->orientation(),
            nominalState.orientation);

    Eigen::Matrix<
        double,
        MeasurementDimension,
        MeasurementDimension> R;

    R.setZero();

    R.block<3, 3>(0, 0) =
        0.05 * 0.05 *
        Eigen::Matrix3d::Identity();

    R.block<3, 3>(3, 3) =
        0.02 * 0.02 *
        Eigen::Matrix3d::Identity();

    const Eigen::Matrix<
        double,
        ErrorStateDimension,
        ErrorStateDimension>& P =
        uncertainty.matrix();

    const Eigen::Matrix<
        double,
        MeasurementDimension,
        MeasurementDimension> S =
        H * P * H.transpose() + R;

    const Eigen::Matrix<
        double,
        ErrorStateDimension,
        MeasurementDimension> K =
        P * H.transpose() *
        S.ldlt().solve(
            Eigen::Matrix<
                double,
                MeasurementDimension,
                MeasurementDimension>::Identity());

    const Eigen::Matrix<
        double,
        ErrorStateDimension,
        1> deltaX =
        K * residual;

    errorState.positionError =
        deltaX.segment<3>(0);

    errorState.velocityError =
        deltaX.segment<3>(3);

    errorState.orientationError =
        deltaX.segment<3>(6);

    errorState.accelerometerBiasError =
        deltaX.segment<3>(9);

    errorState.gyroscopeBiasError =
        deltaX.segment<3>(12);

    nominalState.position +=
        errorState.positionError;

    nominalState.velocity +=
        errorState.velocityError;

    nominalState.orientation =
        applyOrientationError(
            errorState.orientationError,
            nominalState.orientation);

    nominalState.accelerometerBias +=
        errorState.accelerometerBiasError;

    nominalState.gyroscopeBias +=
        errorState.gyroscopeBiasError;

    nominalState.normalizeOrientation();

    const Eigen::Matrix<
        double,
        ErrorStateDimension,
        ErrorStateDimension> I =
        Eigen::Matrix<
            double,
            ErrorStateDimension,
            ErrorStateDimension>::Identity();

    const Eigen::Matrix<
        double,
        ErrorStateDimension,
        ErrorStateDimension> updatedP =
        (I - K * H) * P *
        (I - K * H).transpose()
        + K * R * K.transpose();

    uncertainty.matrix() = updatedP;

    errorState.reset();
}