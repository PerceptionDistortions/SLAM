#pragma once

#include "Backend/SolverStrategy/SolverStrategy.h"

class GaussNewtonStrategy : public SolverStrategy
{
public:
    OptimizationAlgorithm getAlgorithm() const override;
};