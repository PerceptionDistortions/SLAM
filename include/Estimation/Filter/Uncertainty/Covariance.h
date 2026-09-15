#pragma once

#include "Filter/Uncertainty/Uncertainty.h"

#include <Eigen/Core>

class Covariance : public Uncertainty
{
public:
    static constexpr int StateDimension = 9;

    Covariance();

    Eigen::MatrixXd& matrix() override;
    const Eigen::MatrixXd& matrix() const override;

    void reset() override;

    void set(const Eigen::Matrix<double, StateDimension, StateDimension>& covariance);

private:
    Eigen::Matrix<double, StateDimension, StateDimension> _covariance;
};