#pragma once

#include <Eigen/Core>

class Uncertainty
{
public:
    virtual ~Uncertainty() = default;

    virtual Eigen::MatrixXd& matrix() = 0;
    virtual const Eigen::MatrixXd& matrix() const = 0;

    virtual void reset() = 0;
};