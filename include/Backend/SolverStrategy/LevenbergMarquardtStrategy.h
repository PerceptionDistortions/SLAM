#pragma once

#include "Backend/SolverStrategy/SolverStrategy.h"

class LevenbergMarquardtStrategy : public SolverStrategy
{
public:
    OptimizationAlgorithm getAlgorithm() const override;
};