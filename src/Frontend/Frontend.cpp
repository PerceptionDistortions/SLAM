#include "Frontend/Frontend.h"

#include "Frontend/VisualFrontend/Pipeline/VisualFrontend.h"
#include "Frontend/ImuFrontend/ImuFrontend.h"
#include"Frontend/LidarFrontend/LidarFrontend.h"

#include <stdexcept>
#include <utility>

Frontend::Frontend(
    std::unique_ptr<ImuFrontend> imuFrontend,
    std::unique_ptr<VisualFrontend> visualFrontend,
    std::unique_ptr<LidarFrontend> lidarFrontend)
    : _imuFrontend(std::move(imuFrontend)),
      _visualFrontend(std::move(visualFrontend)),
      _lidarFrontend(std::move(lidarFrontend))
{
}

Frontend::~Frontend() = default;

std::unique_ptr<Measurement>
Frontend::processIMU(const ImuData& data)
{
    if (!_imuFrontend)
    {
        throw std::runtime_error(
            "IMU frontend is not available.");
    }

    return _imuFrontend->process(data);
}

std::unique_ptr<Measurement>
Frontend::processMonocular(const MonocularFrame& frame)
{
    if (!_visualFrontend)
    {
        throw std::runtime_error(
            "Visual frontend is not available.");
    }

    return _visualFrontend->process(frame);
}