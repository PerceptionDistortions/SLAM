#include "SLAMSystem/MonocularEKFSLAMSystem.h"

#include "Estimation/Filter/ESEKF/ESEKF.h"
#include "Estimation/Filter/Predictor/ConstantVelocityPredictor.h"
#include "Estimation/Filter/Corrector/MonocularVisualCorrector.h"
#include "Estimation/Measurement/MonocularVisualMeasurement.h"

#include "Frontend/MonocularVisualFrontend.h"
#include "SensorManager/SensorManager.h"

#include <memory>
#include <stdexcept>
#include <utility>


MonocularEKFSLAMSystem::MonocularEKFSLAMSystem(
    const Config& config)
{
    // --------------------------------------------------
    // Compose the concrete SLAM system
    // --------------------------------------------------

    sensor_manager_ =
        std::make_unique<SensorManager>(config);

    frontend_ =
        std::make_unique<MonocularVisualFrontend>(config);

    estimator_ =
        std::make_unique<ESEKF>(
            std::make_unique<ConstantVelocityPredictor>(),
            std::make_unique<MonocularVisualCorrector>());

    // Construct other modules here when implemented.
    //
    // backend_ =
    //     std::make_unique<...>(config);
    //
    // visualizer_ =
    //     std::make_unique<...>(config);
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

    sensor_manager_->initialize();

    frontend_->initialize();

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

    // Monocular system currently processes camera data.
    if (!std::holds_alternative<CameraData>(data))
    {
        return;
    }

    const CameraData& cameraData =
        std::get<CameraData>(data);

    // --------------------------------------------------
    // Sensor data → visual frontend
    // --------------------------------------------------

    MonocularFrame frame;
    frame.camera = cameraData;

    std::unique_ptr<Measurement> measurement =
        frontend_->processMonocular(frame);

    if (!measurement)
    {
        return;
    }

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