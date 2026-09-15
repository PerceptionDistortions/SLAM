#include "Filter/Corrector/MonocularVisualCorrector.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>

namespace
{
    constexpr int ErrorStateDimension = 9;
    constexpr int MeasurementDimension = 6;
}

MonocularVisualMeasurement::MonocularVisualMeasurement(
    double timestamp,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation)
    : _timestamp(timestamp),
      _position(position),
      _orientation(orientation.normalized())
{
}

double MonocularVisualMeasurement::timestamp() const
{
    return _timestamp;
}

const Eigen::Vector3d& MonocularVisualMeasurement::position() const
{
    return _position;
}

const Eigen::Quaterniond& MonocularVisualMeasurement::orientation() const
{
    return _orientation;
}


Eigen::Vector3d MonocularVisualCorrector::orientationError(
    const Eigen::Quaterniond& measured,
    const Eigen::Quaterniond& estimated)
{
    Eigen::Quaterniond delta =
        measured * estimated.conjugate();

    delta.normalize();

    // Ensure shortest rotation.
    if (delta.w() < 0.0)
    {
        delta.coeffs() *= -1.0;
    }

    // Small-angle approximation:
    //
    // q ≈ [1, 0.5 * deltaTheta]
    //
    return 2.0 * delta.vec();
}


Eigen::Quaterniond MonocularVisualCorrector::applyOrientationError(
    const Eigen::Vector3d& deltaTheta,
    const Eigen::Quaterniond& orientation)
{
    const double angle = deltaTheta.norm();

    Eigen::Quaterniond deltaQuaternion;

    if (angle < 1e-12)
    {
        deltaQuaternion = Eigen::Quaterniond::Identity();
    }
    else
    {
        deltaQuaternion =
            Eigen::AngleAxisd(angle, deltaTheta / angle);
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
        dynamic_cast<const MonocularVisualMeasurement*>(&measurement);

    if (!visualMeasurement)
    {
        throw std::invalid_argument(
            "MonocularVisualCorrector received unsupported measurement type.");
    }

    /*
        Error state:

        δx =
        [ δp
          δv
          δθ ]

        Measurement:

        z =
        [ position
          orientation ]

        Therefore:

        H =
        [ I  0  0
          0  0  I ]
    */

    Eigen::Matrix<double, MeasurementDimension, ErrorStateDimension> H;
    H.setZero();

    H.block<3, 3>(0, 0).setIdentity(); // position
    H.block<3, 3>(3, 6).setIdentity(); // orientation

    /*
        Innovation / residual:

        r =
        [ p_measured - p_estimated
          δtheta ]
    */

    Eigen::Matrix<double, MeasurementDimension, 1> residual;

    residual.segment<3>(0) =
        visualMeasurement->position() -
        nominalState.position;

    residual.segment<3>(3) =
        orientationError(
            visualMeasurement->orientation(),
            nominalState.orientation);

    /*
        Measurement noise.

        These values should eventually come from configuration
        or the frontend's visual uncertainty model.
    */

    Eigen::Matrix<double, MeasurementDimension, MeasurementDimension> R;

    R.setZero();

    R.block<3, 3>(0, 0) =
        0.05 * 0.05 *
        Eigen::Matrix3d::Identity();

    R.block<3, 3>(3, 3) =
        0.02 * 0.02 *
        Eigen::Matrix3d::Identity();

    const Eigen::Matrix<double, ErrorStateDimension, ErrorStateDimension>& P =
        uncertainty.matrix();

    /*
        S = HPH^T + R
    */

    Eigen::Matrix<double, MeasurementDimension, MeasurementDimension> S =
        H * P * H.transpose() + R;

    /*
        K = PH^T S^-1
    */

    Eigen::Matrix<double, ErrorStateDimension, MeasurementDimension> K =
        P * H.transpose() *
        S.ldlt().solve(
            Eigen::Matrix<double, MeasurementDimension, MeasurementDimension>::Identity());

    /*
        δx = K r
    */

    Eigen::Matrix<double, ErrorStateDimension, 1> deltaX =
        K * residual;

    errorState.positionError =
        deltaX.segment<3>(0);

    errorState.velocityError =
        deltaX.segment<3>(3);

    errorState.orientationError =
        deltaX.segment<3>(6);

    /*
        Inject error into nominal state.
    */

    nominalState.position +=
        errorState.positionError;

    nominalState.velocity +=
        errorState.velocityError;

    nominalState.orientation =
        applyOrientationError(
            errorState.orientationError,
            nominalState.orientation);

    /*
        Joseph-form covariance update:

        P = (I-KH)P(I-KH)^T + KRK^T

        This is numerically more stable than:
        P = (I-KH)P
    */

    const Eigen::Matrix<double, ErrorStateDimension,
                        ErrorStateDimension> I =
        Eigen::Matrix<double, ErrorStateDimension,
                      ErrorStateDimension>::Identity();

    Eigen::Matrix<double, ErrorStateDimension,
                   ErrorStateDimension> updatedP =
        (I - K * H) * P *
        (I - K * H).transpose()
        + K * R * K.transpose();

    uncertainty.matrix() = updatedP;

    /*
        After injection, the local error state
        is reset to zero.
    */

    errorState.reset();

    nominalState.normalizeOrientation();
}