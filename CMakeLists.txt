cmake_minimum_required(VERSION 3.16)

project(slam_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin CONFIG REQUIRED
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../Pangolin/build
)

add_library(slam_core INTERFACE)

target_include_directories(slam_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_core INTERFACE
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

# Application
add_executable(slam_app
    src/main.cpp

    # Sensors
    src/Sensors/Buffers/BufferManager.cpp

    # Calibration
    src/Sensors/Calibration/Loaders/EurocCalibrationLoader.cpp
    src/Sensors/Calibration/Manager/CalibrationManager.cpp

    # Dataset playback
    src/Sensors/DatasetPlayer/DatasetPlayer.cpp

    # Dataset sources
    src/Sensors/DriverDataSource/EurocCameraSource.cpp
    src/Sensors/DriverDataSource/EurocImuSource.cpp

    # Drivers
    src/Sensors/Drivers/EurocDatasetDriver.cpp

    # Sensor context / manager
    src/Sensors/SensorContext/SensorContext.cpp
    src/Sensors/SensorManager/SensorManager.cpp

    # Visualization
    src/Visualization/PangolinVIsualizer.cpp

    # Estimation
    src/Estimation/Filter/Corrector/MonocularVisualCorrector.cpp
    src/Estimation/Filter/ESEKF/ESEKF.cpp
    src/Estimation/Filter/Predictor/ConstantVelocityPredictor.cpp
    src/Estimation/Filter/State/ErrorState.cpp
    src/Estimation/Filter/State/NominalState.cpp
    src/Estimation/Filter/Uncertainty/Covariance.cpp

    # Measurement
    src/Estimation/Measurement/MonocularVisualMeasurement.cpp

    # Frontend
    src/Frontend/Frontend.cpp

    # SLAM System
    src/SLAMSystem/SLAMSystem.cpp
    src/SLAMSystem/MonocularEKFSLAMSystem.cpp
)

target_include_directories(slam_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Pangolin/components/pango_core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Pangolin/components/pango_display/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Pangolin/components/pango_plot/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Pangolin/components/pango_video/include
)

target_link_libraries(slam_app PRIVATE
    slam_core
    yaml-cpp
    pango_display
)