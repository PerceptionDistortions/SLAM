#pragma once

#include<memory>

#include"Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include"Frontend/ImuFrontend/ImuFrontend.h"
#include"Frontend/LidarFrontend/LidarFrontend.h"
#include"Sensors/Buffers/BufferManager.h"

class FrontendManager
{

public:
    using SensorData = BufferManager::Data;

    //PROCESSING EACH FRONTEND
    FrontendManager(
        std::unique_ptr<ImuFrontend> _imuFrontend,
        std::unique_ptr<VisualFrontend> _visualFrontend,
        std::unique_ptr<LidarFrontend> _lidarFrontend);

    ~FrontendManager();

     bool init();

    // High-level entry point
    std::unique_ptr<Measurement> process(const SensorData& data);

private:
    std::unique_ptr<VisualFrontend> visualFrontend_;
    std::unique_ptr<ImuFrontend> imuFrontend_;
    std::unique_ptr<LidarFrontend> lidarFrontend_;

    std::unique_ptr<Measurement> processVisual(const MonocularFrame& frame);
    std::unique_ptr<Measurement> processIMU(const ImuData& data);

    bool initialized_{false};

};