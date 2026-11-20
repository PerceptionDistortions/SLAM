#include "Backend/SolverStrategy/DoglegStrategy.h"

OptimizationAlgorithm DoglegStrategy::getAlgorithm() const
{
    return OptimizationAlgorithm::Dogleg;
}