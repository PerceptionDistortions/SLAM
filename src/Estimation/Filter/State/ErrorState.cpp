#include "Filter/State/ErrorState.h"

void ErrorState::reset()
{
    positionError.setZero();
    velocityError.setZero();
    orientationError.setZero();
}