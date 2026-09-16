//USED IN BOTH ESTIMATOR AND OPTIMIZERS
//MEASUREMENT: RAW/PROCESSED SENSOR DATA AVAILABLE TO ESTIMATOR
#pragma once

#include "Estimation/Measurement/MeasurementRole.h"

class Measurement
{
public:
    virtual ~Measurement() = default;
    virtual double timestamp() const = 0;
    virtual MeasurementRole role() const = 0;
};