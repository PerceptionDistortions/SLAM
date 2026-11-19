#pragma once
#include<memory>

#include"Backend/Solver/OptimizationSolver.h"
#include"Backend/SolverStrategy/SolverStrategy.h"
#include"Backend/OptimizationProblem/OptimizationProblem.h"

class CeresSolver : public OptimizationSolver
{
public:
    explicit CeresSolver(std::unique_ptr<SolverStrategy> strategy);
    bool solve(OptimizationProblem& problem) override;

private:
    std::unique_ptr<SolverStrategy> strategy_;
};