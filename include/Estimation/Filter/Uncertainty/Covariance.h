#pragma once

#include "Estimation/Filter/Uncertainty/Uncertainty.h"

#include <Eigen/Core>

class Covariance : public Uncertainty
{
public:
    static constexpr int StateDimension = 15;

    Covariance();

    Eigen::MatrixXd& matrix() override;
    const Eigen::MatrixXd& matrix() const override;

    void reset() override;

    void set(
        const Eigen::Matrix<double,
                            StateDimension,
                            StateDimension>& covariance);

private:
    Eigen::MatrixXd _covariance;
};