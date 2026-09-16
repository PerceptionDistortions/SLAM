#include "Estimation/Filter/State/NominalState.h"

void NominalState::normalizeOrientation()
{
    orientation.normalize();
}