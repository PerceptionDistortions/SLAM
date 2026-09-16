#pragma once

#include "Estimation/Filter/Corrector/Corrector.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

class MonocularVisualCorrector : public Corrector
{
public:
    void correct(
        NominalState& nominalState,
        ErrorState& errorState,
        Uncertainty& uncertainty,
        const Measurement& measurement) override;

private:
    static Eigen::Vector3d orientationError(
        const Eigen::Quaterniond& measured,
        const Eigen::Quaterniond& estimated);

    static Eigen::Quaterniond applyOrientationError(
        const Eigen::Vector3d& deltaTheta,
        const Eigen::Quaterniond& orientation);
};