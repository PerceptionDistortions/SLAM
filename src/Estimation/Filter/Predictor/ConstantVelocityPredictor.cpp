#include "Filter/Predictor/ConstantVelocityPredictor.h"

void ConstantVelocityPredictor::predict(
    NominalState& state,
    double dt)
{
    state.position += state.velocity * dt;

    // Constant velocity model:
    // velocity remains unchanged.
    // Orientation remains unchanged.

    state.normalizeOrientation();
}