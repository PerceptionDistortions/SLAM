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

std::unique_ptr<Measurement>
FrontendManager::processIMU(const ImuData& data)
{
    if (!imuFrontend_)
    {
        throw std::runtime_error("IMU frontend is not available.");
    }

    return imuFrontend_->process(data);
}

std::unique_ptr<Measurement>
FrontendManager::processVisual(const MonocularFrame& frame)
{
    if (!visualFrontend_)
    {
        throw std::runtime_error("Visual frontend is not available.");
    }
    return visualFrontend_->process(frame);
}