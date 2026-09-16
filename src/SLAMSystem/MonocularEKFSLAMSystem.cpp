#include "SLAMSystem/MonocularEKFSLAMSystem.h"

#include "Estimation/Filter/ESEKF/ESEKF.h"
#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"
#include "Estimation/Filter/Corrector/MonocularVisualCorrector.h"
#include "Estimation/Measurement/MonocularVisualMeasurement.h"

#include <memory>
#include <stdexcept>
#include <utility>

void MonocularEKFSLAMSystem::init()
{
    estimator_ =
        std::make_unique<ESEKF>(
            std::make_unique<ConstantVelocityPredictor>(),
            std::make_unique<MonocularVisualCorrector>());

    initialized_ = true;
    running_ = false;
}

void MonocularEKFSLAMSystem::update()
{
    if (!initialized_)
    {
        throw std::runtime_error(
            "MonocularEKFSLAMSystem must be initialized "
            "before update.");
    }

    SensorManager::Data data;

    if (!sensor_manager_->popNext(data))
    {
        running_ = false;
        return;
    }

    // This monocular SLAM system only processes camera data.
    if (!std::holds_alternative<CameraData>(data))
    {
        return;
    }

    const CameraData& cameraData =
        std::get<CameraData>(data);

    // Convert CameraData into the visual frontend's input.
    MonocularFrame frame;
    frame.camera = cameraData;

    // Frontend performs the complete visual processing
    // and returns an estimator-level measurement.
    std::unique_ptr<Measurement> measurement =
        frontend_->processMonocular(frame);

    if (!measurement)
    {
        return;
    }

    // The first valid visual measurement establishes
    // the initial estimator state.
    if (!estimator_initialized_)
    {
        const auto* visualMeasurement =
            dynamic_cast<const MonocularVisualMeasurement*>(
                measurement.get());

        if (!visualMeasurement)
        {
            throw std::runtime_error(
                "MonocularEKFSLAMSystem expected "
                "MonocularVisualMeasurement.");
        }

        StateEstimate initialState;

        initialState.timestamp =
            visualMeasurement->timestamp();

        initialState.position =
            visualMeasurement->position();

        initialState.velocity.setZero();

        initialState.orientation =
            visualMeasurement->orientation();

        initialState.accelerometerBias.setZero();

        initialState.gyroscopeBias.setZero();

        estimator_->initialize(initialState);

        estimator_initialized_ = true;

        return;
    }

    // Every subsequent visual measurement performs:
    //
    // prediction → visual correction
    //
    // because MonocularVisualMeasurement::role()
    // returns PredictionAndCorrection.
    estimator_->process(*measurement);
}

void MonocularEKFSLAMSystem::run()
{
    if (!initialized_)
    {
        throw std::runtime_error(
            "MonocularEKFSLAMSystem must be initialized "
            "before run.");
    }

    running_ = true;

    while (running_)
    {
        update();
    }
}

void MonocularEKFSLAMSystem::shutdown()
{
    running_ = false;
}

StateEstimate MonocularEKFSLAMSystem::getState() const
{
    if (!estimator_)
    {
        throw std::runtime_error(
            "Estimator is not initialized.");
    }

    return estimator_->getEstimate();
}