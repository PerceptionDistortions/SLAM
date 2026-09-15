#pragma once

#include "Filter/Corrector/Corrector.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

class MonocularVisualMeasurement : public Measurement
{
public:
    MonocularVisualMeasurement(
        double timestamp,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation);

    double timestamp() const override;

    const Eigen::Vector3d& position() const;
    const Eigen::Quaterniond& orientation() const;

private:
    double _timestamp;
    Eigen::Vector3d _position;
    Eigen::Quaterniond _orientation;
};


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