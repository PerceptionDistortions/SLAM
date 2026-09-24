#include "SLAMSystem/MonocularEKFSLAMSystem.h"

#include "Estimation/Filter/ESEKF/ESEKF.h"
#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"
#include "Estimation/Filter/Corrector/MonocularVisualCorrector.h"
#include "Estimation/Measurement/MonocularVisualMeasurement.h"

#include "Frontend/VisualFrontend/Pipeline/MonocularVisualFrontend.h"
#include"Factory/SLAMSystemDependencies.h"

#include <memory>
#include <stdexcept>
#include <utility>


MonocularEKFSLAMSystem::MonocularEKFSLAMSystem(
    const SystemConfig& config,
    SLAMSystemDependencies dependencies)
{
    sensor_manager_ =std::move(dependencies.sensorManager);
    frontend_ =std::move(dependencies.frontend);
    estimator_ =std::move(dependencies.estimator);
    visualizer_ =std::move(dependencies.visualizer);
    backend_ =std::move(dependencies.backend);
    loop_closure_ =std::move(dependencies.loopClosure);
}


void MonocularEKFSLAMSystem::init()
{
    if (initialized_)
    {
        return;
    }

    // --------------------------------------------------
    // Runtime initialization
    // --------------------------------------------------

    sensor_manager_->init();

    frontend_->init();

    estimator_initialized_ = false;
    running_ = false;

    initialized_ = true;
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

    std::unique_ptr<Measurement> measurement =frontend_->process(data);
    if (!measurement) return;

    // --------------------------------------------------
    // First valid measurement initializes estimator
    // --------------------------------------------------

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

        initialState.timestamp =visualMeasurement->timestamp();
        initialState.position =visualMeasurement->position();
        initialState.velocity.setZero();
        initialState.orientation =visualMeasurement->orientation();
        initialState.accelerometerBias.setZero();
        initialState.gyroscopeBias.setZero();
        estimator_->initialize(initialState);
        estimator_initialized_ = true;
        return;
    }

    // --------------------------------------------------
    // Subsequent measurements
    //
    // MonocularVisualMeasurement:
    // PredictionAndCorrection
    // --------------------------------------------------

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
            "Estimator is not constructed.");
    }

    return estimator_->getEstimate();
}