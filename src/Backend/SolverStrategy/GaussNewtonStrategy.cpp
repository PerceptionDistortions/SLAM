#include "Backend/SolverStrategy/GaussNewtonStrategy.h"

OptimizationAlgorithm GaussNewtonStrategy::getAlgorithm() const
{
    return OptimizationAlgorithm::GaussNewton;
}