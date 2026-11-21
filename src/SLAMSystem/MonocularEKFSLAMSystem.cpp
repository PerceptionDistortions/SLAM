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
//NO BACKEND INIT REQUIRED
//NO MAP MANAGER INIT: HANDLED BY FACTORY
bool MonocularEKFSLAMSystem::init()
{
    if (initialized_) return true;

    //INIT AND START SENSOR
    if(!sensor_manager_->init()) return false;
    if(!sensor_manager_->start()) return false;

    //INIT FRONTEND
    if(!frontend_->init()) return false;

    //INIT LOOP CLOSURE

    //INIT VISUALIZER

    running_ = false;

    initialized_ = true;

    return true;
}

//ONE SLAM ITERATION
bool MonocularEKFSLAMSystem::update()
{
    //SLAM SYSTEM INIT CHECK
    if (!initialized_) throw std::runtime_error("Monocular SLAM System not initialized.");

    //BUFFER DATA VALIDATION THAT WAS SUPPLIED
    SensorManager::Data data;
    if (!sensor_manager_->popNext(data)) return false;

    //FRONTEND END EXISTS AND VALID?
    if (!frontend_) return false; 

    //OUTPUT OF THE VISUAL FRONTEND
    std::unique_ptr<Measurement> measurement;

    //------------------------------------------------------------------------------------

    // ONLY CAMERA DATA ALLOWED: MONO SLAM
    if (std::holds_alternative<CameraData>(data))
    {
        const auto& cameraData =std::get<CameraData>(data);

        //DATA COMES FROM SPECIFIED CAMERA?
        if (cameraData.sensor_id != camera_name_) return false;

        //INIT FIRST STATE
        if (!estimator_initialized_)
        {
            StateEstimate initialState;
            initialState.timestamp = static_cast<double>(cameraData.timestamp) * 1e-9;
            initialState.position = Eigen::Vector3d::Zero();
            initialState.velocity = Eigen::Vector3d::Zero();
            initialState.orientation = Eigen::Quaterniond::Identity();
            estimator_->initialize(initialState);
            estimator_initialized_ = true;
        }
        //WE DO NOT KEEP IT IN INIT
        //TIME STAMP OF FIRST FRAME IS REQUIRED TO INIT
        //INIT ESTIMATOR: THE FIRST FRAME IS ORIGIN IN MONOCULAR
        //CREATE AN STATE AND INIT ESTIMATOR, NOT POSSIBLE TO DIRECTLY ASSIGN
        //COVARIANCE NEEDS TO BE INIT, INITIAL UNCETAINTY MATRIX
        //DONE IN UPDATE BECAUSE OF TIMESTAMP COMPATIBILITY


        // CONTROLLED DATASET PLAYBACK
        if (keyBasedControlledPlayback_)
        {
            char key;
            std::cout << "\nPress P + Enter to process next frame..."<< std::endl;
            do
            {
                if (!(std::cin >> key)) return false;
            }
            while (key != 'p' && key != 'P');
        }

        //CAMERA TIMESTAMP
        std::cout<<"\n"
            << "[CAMERA] Found "
            << cameraData.sensor_id
            << " | timestamp=" << cameraData.timestamp
            << std::endl;

        // FRONTEND DATA IS A VARIANT: CONVERT CAMERA FRONTEND TO IT
        //REQUIRED TO PROCESS IN GENERIC ABSTRACTION
        FrontendData frontendData = convertCameraDataToFrontendData(cameraData);
        measurement = frontend_->process(frontendData);
        if (!measurement)
        {
            std::cout << "Frontend returned null measurement" << std::endl;
            return false;
        }

        //IS ESTIMATOR INIT?
        if (!estimator_ || !estimator_initialized_) return false;

        //PROCESS IN ESTIMATOR
        estimator_->process(*measurement);
    }

    //IF NO CAMERA DATA FROM CORRECT CAMERA SKIP
    else
    {
        // This monocular SLAM pipeline does not handle IMU
        return false;
    }

    //BACKEND OPTIMIZATION AND LOOP CLOSURE CHECK
    std::cout<<"SLAM System update complete."<<std::endl;
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

//THE TIMESTAMO NEEDS TO BE CONVERTED TO FRONTEND DATA
FrontendData MonocularEKFSLAMSystem::convertCameraDataToFrontendData(
    const CameraData& cameraData) const
{
    MonocularFrame frame;

    // frame.timestamp = cameraData.timestamp;
    frame.timestamp = static_cast<double>(cameraData.timestamp) * 1e-9;
    frame.camera.image = cameraData.image;

    return frame;
}
