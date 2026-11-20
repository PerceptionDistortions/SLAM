#pragma once

#include "Backend/SolverStrategy/SolverStrategy.h"

class DoglegStrategy : public SolverStrategy
{
public:
    OptimizationAlgorithm getAlgorithm() const override;
};