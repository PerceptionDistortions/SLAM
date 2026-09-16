#include "Estimation/Measurement/MonocularVisualMeasurement.h"

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

MeasurementRole MonocularVisualMeasurement::role() const
{
    return MeasurementRole::PredictionAndCorrection;
}

const Eigen::Vector3d&
MonocularVisualMeasurement::position() const
{
    return _position;
}

const Eigen::Quaterniond&
MonocularVisualMeasurement::orientation() const
{
    return _orientation;
}