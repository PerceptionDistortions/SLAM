#pragma once
#include<memory>

#include"Backend/Solver/OptimizationSolver.h"
#include"Backend/SolverStrategy/SolverStrategy.h"
#include"Backend/OptimizationProblem/OptimizationProblem.h"

#include <string>

struct CeresSolverConfig {
    int maxNumIterations{50};
    double maxSolverTimeSeconds{2.0};
    double functionTolerance{1.0e-6};
    double gradientTolerance{1.0e-10};
    double parameterTolerance{1.0e-8};
    int numThreads{1};
    std::string linearSolver{"DENSE_QR"};
};

class CeresSolver : public OptimizationSolver
{
public:
    explicit CeresSolver(std::unique_ptr<SolverStrategy> strategy,
        CeresSolverConfig config = {});
    bool solve(OptimizationProblem& problem) override;

private:
    std::unique_ptr<SolverStrategy> strategy_;
    CeresSolverConfig config_;
};
