#include "Estimation/Filter/Uncertainty/Covariance.h"

Covariance::Covariance()
{
    reset();
}

Eigen::MatrixXd& Covariance::matrix()
{
    return _covariance;
}

const Eigen::MatrixXd& Covariance::matrix() const
{
    return _covariance;
}

void Covariance::reset()
{
    _covariance =
        Eigen::MatrixXd::Identity(
            StateDimension,
            StateDimension);
}

void Covariance::set(
    const Eigen::Matrix<double,
                        StateDimension,
                        StateDimension>& covariance)
{
    _covariance = covariance;
}