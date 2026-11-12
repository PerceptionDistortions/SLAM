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

#include <iostream>
#include <thread>
#include <chrono>

//CONSTRUCTOR
//SLAM FACTORY HAS ALREADY CREATED EVERYTHING
MonocularEKFSLAMSystem::MonocularEKFSLAMSystem(
    SLAMSystemDependencies dependencies,
    const std::string& cameraName,
    bool keyBasedControlledPlayback)
    : camera_name_(cameraName),
      keyBasedControlledPlayback_(keyBasedControlledPlayback)
{
    sensor_manager_ = std::move(dependencies.sensorManager);
    map_manager_ = std::move(dependencies.mapManager);
    frontend_ = std::move(dependencies.frontend);
    estimator_ = std::move(dependencies.estimator);
    visualizer_ = std::move(dependencies.visualizer);
    backend_ = std::move(dependencies.backend);
    loop_closure_ = std::move(dependencies.loopClosure);
}

//INITIAL STATE OF ESTIMATOR
bool MonocularEKFSLAMSystem::init()
{
    if (initialized_) return true;

    //INIT AND START SENSOR
    if(!sensor_manager_->init()) return false;
    if(!sensor_manager_->start()) return false;

    //INIT ESTIMATOR: THE FIRST FRAME IS ORIGIN IN MONOCULAR
    //CREATE AN STATE AND INIT ESTIMATOR, NOT POSSIBLE TO DIRECTLY ASSIGN
    //COVARIANCE NEEDS TO BE INIT, INITIAL UNCETAINTY MATRIX
    StateEstimate initialState;
    initialState.timestamp = 0.0;
    initialState.position = Eigen::Vector3d::Zero();
    initialState.velocity = Eigen::Vector3d::Zero();
    initialState.orientation = Eigen::Quaterniond::Identity();

    estimator_->initialize(initialState);
    estimator_initialized_ = true;

    //INIT FRONTEND
    if(!frontend_->init()) return false;

    //INIT MAP MANAGER

    //INIT BACKEND

    //INIT LOOP CLOSURE

    //INIT VISUALIZER

    running_ = false;

    initialized_ = true;

    return true;
}

//ONE SLAM ITERATION
bool MonocularEKFSLAMSystem::update()
{
    //INIT OF SLAM SYSTEM: CHECK
    if (!initialized_) throw std::runtime_error("Monocular SLAM System not initialized.");

    //GET THE NEXT DATA FROM SENSOR BUFFER
    SensorManager::Data data;
    if (!sensor_manager_->popNext(data)) return false;

    //CHECK IF FRONTEND IS DEFINED
    if (!frontend_)
    {
        std::cout << "Frontend not found" << std::endl;
        return false;
    }

    //MEASUREMENT WILL HOLD THE DATA OF THE VISUAL FRONTEND
    std::unique_ptr<Measurement> measurement;

    // ONLY CAMERA DATA ALLOWED: MONO SLAM
    if (std::holds_alternative<CameraData>(data))
    {
        const auto& cameraData =std::get<CameraData>(data);

        // Only process the configured camera
        if (cameraData.sensor_id != camera_name_) return false;

        // CONTROLLED DATASET PLAYBACK
        if (keyBasedControlledPlayback_)
        {
            char key;

            std::cout << "\nPress P + Enter to process next frame..."
                    << std::endl;

            do
            {
                if (!(std::cin >> key))
                    return false;
            }
            while (key != 'p' && key != 'P');
        }

        //CAMERA TIMESTAMP
        std::cout<<"\n"
            << "[CAMERA] Found "
            << cameraData.sensor_id
            << " | timestamp=" << cameraData.timestamp
            << std::endl;

        // Convert CameraData -> FrontendData
        FrontendData frontendData = convertCameraDataToFrontendData(cameraData);

        // DATA CONVERTED
        measurement = frontend_->process(frontendData);
        if (!measurement)
        {
            std::cout << "Frontend returned null measurement" << std::endl;
            return false;
        }
        if (!estimator_ || !estimator_initialized_) return false;

        //THE MEASUREMENT TYPE 
        estimator_->process(*measurement);
    }
    else
    {
        // This monocular SLAM pipeline does not handle IMU
        return false;
    }

    //THE RESULTING MEASURMENT IS PASSED TO THE ESTIMATOR: EKF
    

    //PASS THE VISUAL FRONTEND DATA TO ESTIMATOR

    //BACKEND OPTIMIZATION AND LOOP CLOSURE CHECK

    return true;
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

    std::cout << "\n=== SLAM RUN ===" << std::endl;

    std::size_t count = 0;

    while (sensor_manager_->isRunning())
    {
        if (update())
        {
            ++count;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Drain data already buffered after players finish.
    while (update())
    {
        ++count;
    }

    std::cout
        << "Total sensor samples consumed = "
        << count
        << std::endl;

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

FrontendData MonocularEKFSLAMSystem::convertCameraDataToFrontendData(
    const CameraData& cameraData) const
{
    MonocularFrame frame;

    frame.timestamp = cameraData.timestamp;
    frame.camera.image = cameraData.image;

    return frame;
}
