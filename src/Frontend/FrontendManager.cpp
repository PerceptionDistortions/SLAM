#include "Frontend/FrontendManager.h"

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include "Frontend/ImuFrontend/ImuFrontend.h"
#include "Frontend/LidarFrontend/LidarFrontend.h"

#include <stdexcept>
#include <utility>

FrontendManager::FrontendManager(
    std::unique_ptr<ImuFrontend> imuFrontend,
    std::unique_ptr<VisualFrontend> visualFrontend,
    std::unique_ptr<LidarFrontend> lidarFrontend)
    : imuFrontend_(std::move(imuFrontend)),
      visualFrontend_(std::move(visualFrontend)),
      lidarFrontend_(std::move(lidarFrontend))
{
}

FrontendManager::~FrontendManager() = default;

bool FrontendManager::init()
{
    if (initialized_)
        return true;

    if (visualFrontend_)
    {
        if (!visualFrontend_->init())
            return false;
    }

    if (imuFrontend_)
    {
        if (!imuFrontend_->init())
            return false;
    }

    if (lidarFrontend_)
    {
        if (!lidarFrontend_->init())
            return false;
    }

    initialized_ = true;
    return true;
}

std::unique_ptr<Measurement>
FrontendManager::process(const SensorData& data)
{
    if (!initialized_)
    {
        throw std::runtime_error(
            "FrontendManager must be initialized before processing.");
    }

    return std::visit(
        [this](const auto& sensorData)
        -> std::unique_ptr<Measurement>
        {
            using T = std::decay_t<decltype(sensorData)>;

            if constexpr (std::is_same_v<T, ImuData>)
            {
                return processIMU(sensorData);
            }
            else if constexpr (
                std::is_same_v<T, MonocularFrame> ||
                std::is_same_v<T, StereoFrame>)
            {
                VisualData visualData = sensorData;
                return processVisual(visualData);
            }
            else
            {
                return nullptr;
            }
        },
        data);
}


// ---------------------------------------------------------
// VISUAL PROCESSING
// ---------------------------------------------------------

std::unique_ptr<Measurement>
FrontendManager::processVisual(const VisualData& data)
{
    if (!visualFrontend_)
    {
        throw std::runtime_error(
            "Visual frontend is not available.");
    }

    return visualFrontend_->processVisual(data);
}


// ---------------------------------------------------------
// IMU PROCESSING
// ---------------------------------------------------------

std::unique_ptr<Measurement>
FrontendManager::processIMU(const ImuData& data)
{
    if (!imuFrontend_)
    {
        throw std::runtime_error(
            "IMU frontend is not available.");
    }

    return imuFrontend_->processImu(data);
}