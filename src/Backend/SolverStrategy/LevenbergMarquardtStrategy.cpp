#include "Backend/SolverStrategy/LevenbergMarquardtStrategy.h"

OptimizationAlgorithm LevenbergMarquardtStrategy::getAlgorithm() const
{
    return OptimizationAlgorithm::LevenbergMarquardt;
}