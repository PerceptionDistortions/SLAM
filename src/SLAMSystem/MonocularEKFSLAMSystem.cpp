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

//CONSTRUCTOR
//SLAM FACTORY HAS ALREADY CREATED EVERYTHING
MonocularEKFSLAMSystem::MonocularEKFSLAMSystem(const SystemConfig& config,
    SLAMSystemDependencies dependencies)
{
    sensor_manager_ =std::move(dependencies.sensorManager);
    frontend_ =std::move(dependencies.frontend);
    estimator_ =std::move(dependencies.estimator);
    visualizer_ =std::move(dependencies.visualizer);
    backend_ =std::move(dependencies.backend);
    loop_closure_ =std::move(dependencies.loopClosure);
}


bool MonocularEKFSLAMSystem::init()
{
    if (initialized_)
    {
        return true;
    }

    //INIT AND START SENSOR
    if(!sensor_manager_->init()) return false;
    if(!sensor_manager_->start()) return false;

    //INIT MAP MANAGER

    //INIT FRONTEND
    if(!frontend_->init()) return false;

    //INIT BACKEND

    //INIT LOOP CLOSURE

    //INIT VISUALIZER

    estimator_initialized_ = false;
    running_ = false;

    initialized_ = true;

    return true;
}


bool MonocularEKFSLAMSystem::update()
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
        return false;
    }

    std::visit(
        [](const auto& sensorData)
        {
            using T = std::decay_t<decltype(sensorData)>;

            if constexpr (std::is_same_v<T, CameraData>)
            {
                std::cout
                    << "[CAMERA]"
                    << " sensor=" << sensorData.sensor_id
                    << " timestamp=" << sensorData.timestamp
                    << std::endl;
            }
            else if constexpr (std::is_same_v<T, ImuData>)
            {
                std::cout
                    << "[IMU]"
                    << " sensor=" << sensorData.sensor_id
                    << " timestamp=" << sensorData.timestamp
                    << std::endl;
            }
        },
        data);

    return true;
    // if (!initialized_)
    // {
    //     throw std::runtime_error(
    //         "MonocularEKFSLAMSystem must be initialized "
    //         "before update.");
    // }

    // SensorManager::Data data;

    // if (!sensor_manager_->popNext(data))
    // {
    //     running_ = false;
    //     return;
    // }

    // std::unique_ptr<Measurement> measurement =frontend_->process(data);
    // if (!measurement) return;

    // --------------------------------------------------
    // First valid measurement initializes estimator
    // --------------------------------------------------

    // if (!estimator_initialized_)
    // {
    //     const auto* visualMeasurement =
    //         dynamic_cast<const MonocularVisualMeasurement*>(
    //             measurement.get());

    //     if (!visualMeasurement)
    //     {
    //         throw std::runtime_error(
    //             "MonocularEKFSLAMSystem expected "
    //             "MonocularVisualMeasurement.");
    //     }

    //     StateEstimate initialState;

    //     initialState.timestamp =visualMeasurement->timestamp();
    //     initialState.position =visualMeasurement->position();
    //     initialState.velocity.setZero();
    //     initialState.orientation =visualMeasurement->orientation();
    //     initialState.accelerometerBias.setZero();
    //     initialState.gyroscopeBias.setZero();
    //     estimator_->initialize(initialState);
    //     estimator_initialized_ = true;
    //     return;
    // }

    // // --------------------------------------------------
    // // Subsequent measurements
    // //
    // // MonocularVisualMeasurement:
    // // PredictionAndCorrection
    // // --------------------------------------------------

    // estimator_->process(*measurement);
}


void MonocularEKFSLAMSystem::run()
{
    if (!initialized_)
    {
        throw std::runtime_error("MonocularEKFSLAMSystem must be initialized "
            "before run.");
    }

    running_ = true;

    // NORMAL PLAYBACK
    while (sensor_manager_->isRunning())
    {
        if (!update())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // DRAIN REMAINING DATA
    while (update())
    {
    }

    running_ = false;
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