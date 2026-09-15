//USED IN BOTH ESTIMATOR AND OPTIMIZERS
//MEASUREMENT: RAW/PROCESSED SENSOR DATA AVAILABLE TO ESTIMATOR
#pragma once

class Measurement
{
public:
    virtual ~Measurement() = default;
    virtual double timestamp() const = 0;
};