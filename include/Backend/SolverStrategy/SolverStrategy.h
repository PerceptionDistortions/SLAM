#pragma once

#include"AliasesEnumsTypes/OptimizationAlgorithm.h"

class SolverStrategy
{
public:
    virtual ~SolverStrategy() = default;
    virtual OptimizationAlgorithm getAlgorithm() const = 0;
};