#pragma once

#include"Backend/OptimizationProblem/OptimizationProblem.h"

class OptimizationSolver
{
public:
    virtual ~OptimizationSolver() = default;
    virtual bool solve(OptimizationProblem& problem) = 0;
};