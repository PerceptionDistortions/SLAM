#pragma once

#include "Estimation/Measurement/Measurement.h"

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

    MeasurementRole role() const override;

    const Eigen::Vector3d& position() const;

    const Eigen::Quaterniond& orientation() const;

private:
    double _timestamp{0.0};

    Eigen::Vector3d _position{
        Eigen::Vector3d::Zero()
    };

    Eigen::Quaterniond _orientation{
        Eigen::Quaterniond::Identity()
    };
};