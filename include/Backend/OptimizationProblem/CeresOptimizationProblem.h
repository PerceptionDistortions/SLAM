#pragma once

#include "Backend/OptimizationProblem/OptimizationProblem.h"
#include <ceres/ceres.h>

class CeresOptimizationProblem : public OptimizationProblem
{
public:
    ceres::Problem& getProblem() { return problem_; }
    const ceres::Problem& getProblem() const { return problem_; }

private:
    ceres::Problem problem_;
};