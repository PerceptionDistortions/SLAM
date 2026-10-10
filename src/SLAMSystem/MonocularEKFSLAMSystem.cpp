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
MonocularEKFSLAMSystem::MonocularEKFSLAMSystem(
    SLAMSystemDependencies dependencies,
    const std::string& cameraName)
    : camera_name_(cameraName)
{
    sensor_manager_ = std::move(dependencies.sensorManager);
    frontend_ = std::move(dependencies.frontend);
    estimator_ = std::move(dependencies.estimator);
    visualizer_ = std::move(dependencies.visualizer);
    backend_ = std::move(dependencies.backend);
    loop_closure_ = std::move(dependencies.loopClosure);
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

//ONE SLAM ITERATION
// bool MonocularEKFSLAMSystem::update()
// {
//     //CHECK INIT AND ATTEMPT POPING FROM BUFFER
//     if (!initialized_) throw std::runtime_error("Monocular SLAM System not initialized.");
//     SensorManager::Data data;
//     if (!sensor_manager_->popNext(data)) return false;

//     //ONLY 1 CAMERA DATA ALLOWED: MONO SLAM
//     std::visit([this](const auto& sensorData)
//     {
//         using T = std::decay_t<decltype(sensorData)>;
//         if constexpr (std::is_same_v<T, CameraData>)
//         {
//             if (sensorData.sensor_id == camera_name_)
//             {
//                 std::cout
//                     << "[CAMERA]"
//                     << " sensor=" << sensorData.sensor_id
//                     << " timestamp=" << sensorData.timestamp
//                     << std::endl;
//             }
//         }
//     },
//     data);

//     //FOR THE GIVEN SENSOR DATA, PREPROCESS FRONTEND
//     //VARINAT OF SENSOR DATA DETERMINES WHAT FRONTEND TO CHOOSE
//     if (!frontend_)
//     {
//         std::cout << "Frontend not found" << std::endl;
//         return false;
//     }

//     std::unique_ptr<Measurement> measurement;



//     if (!measurement)
//     {
//         std::cout << "Frontend returned null measurement" << std::endl;
//         return false;
//     }


//     return true;
    

//     // std::unique_ptr<Measurement> measurement =frontend_->process(data);
//     // if (!measurement) return;

//     // --------------------------------------------------
//     // First valid measurement initializes estimator
//     // --------------------------------------------------

//     // if (!estimator_initialized_)
//     // {
//     //     const auto* visualMeasurement =
//     //         dynamic_cast<const MonocularVisualMeasurement*>(
//     //             measurement.get());

//     //     if (!visualMeasurement)
//     //     {
//     //         throw std::runtime_error(
//     //             "MonocularEKFSLAMSystem expected "
//     //             "MonocularVisualMeasurement.");
//     //     }

//     //     StateEstimate initialState;

//     //     initialState.timestamp =visualMeasurement->timestamp();
//     //     initialState.position =visualMeasurement->position();
//     //     initialState.velocity.setZero();
//     //     initialState.orientation =visualMeasurement->orientation();
//     //     initialState.accelerometerBias.setZero();
//     //     initialState.gyroscopeBias.setZero();
//     //     estimator_->initialize(initialState);
//     //     estimator_initialized_ = true;
//     //     return;
//     // }

//     // // --------------------------------------------------
//     // // Subsequent measurements
//     // //
//     // // MonocularVisualMeasurement:
//     // // PredictionAndCorrection
//     // // --------------------------------------------------

//     // estimator_->process(*measurement);
// }

bool MonocularEKFSLAMSystem::update()
{
    if (!initialized_) throw std::runtime_error("Monocular SLAM System not initialized.");
    SensorManager::Data data;
    if (!sensor_manager_->popNext(data)) return false;

    if (!frontend_)
    {
        std::cout << "Frontend not found" << std::endl;
        return false;
    }

    std::unique_ptr<Measurement> measurement;

    // ONLY CAMERA DATA ALLOWED: MONO SLAM
    if (std::holds_alternative<CameraData>(data))
    {
        const auto& cameraData =std::get<CameraData>(data);

        // Only process the configured camera
        if (cameraData.sensor_id != camera_name_) return false;

        std::cout<<"\n"
            << "[CAMERA] Found "
            << cameraData.sensor_id
            << " | timestamp=" << cameraData.timestamp
            << std::endl;

        // Convert CameraData -> FrontendData
        FrontendData frontendData = convertCameraDataToFrontendData(cameraData);

        // Send converted data to frontend
        measurement = frontend_->process(frontendData);
    }
    else
    {
        // This monocular SLAM pipeline does not handle IMU
        return false;
    }

    //THE RESULTING MEASURMENT IS PASSED TO THE ESTIMATOR: EKF
    if (!measurement)
    {
        std::cout << "Frontend returned null measurement" << std::endl;
        return false;
    }

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