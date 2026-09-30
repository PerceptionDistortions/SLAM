#include "Sensors/Drivers/EurocDatasetDriver.h"

#include <filesystem>
#include <utility>

EurocDatasetDriver::EurocDatasetDriver(
    const std::string& dataset_path,
    const std::string& sensor_id,
    const std::string& sensor_type,
    const std::string& data_path,
    const std::string& csv,
    BufferManager& buffer_manager,
    const PlaybackConfig& playback_config)
    : dataset_path_(dataset_path),
      sensor_id_(sensor_id),
      sensor_type_(sensor_type),
      data_path_(data_path),
      csv_(csv),
      buffer_manager_(buffer_manager),
      playback_config_(playback_config)
{
}

EurocDatasetDriver::~EurocDatasetDriver()
{
    stop();
}

bool EurocDatasetDriver::createSource()
{
    if (sensor_type_ == "camera")
    {
        source_ =
            std::make_unique<EurocCameraSource>(
                dataset_path_,
                sensor_id_,
                data_path_,
                csv_);

        return true;
    }

    if (sensor_type_ == "imu")
    {
        source_ =
            std::make_unique<EurocImuSource>(
                dataset_path_,
                sensor_id_,
                csv_);

        return true;
    }

    return false;
}

bool EurocDatasetDriver::init()
{
    if (state_ != DriverState::Created) return false;
    
    if (!std::filesystem::exists(dataset_path_))
    {
        state_ = DriverState::Error;
        return false;
    }

    if (!createSource())
    {
        state_ = DriverState::Error;
        return false;
    }

    if (!source_->init())
    {
        state_ = DriverState::Error;
        return false;
    }

    player_ =
        std::make_unique<DatasetPlayer>(
            std::move(source_),
            buffer_manager_,
            playback_config_,
            sensor_id_);

    state_ = DriverState::Ready;

    return true;
}

bool EurocDatasetDriver::start()
{
    if (state_ != DriverState::Ready)
        return false;

    if (!player_)
    {
        state_ = DriverState::Error;
        return false;
    }

    if (!player_->start())
    {
        state_ = DriverState::Error;
        return false;
    }

    state_ = DriverState::Running;

    return true;
}

bool EurocDatasetDriver::stop()
{
    if (state_ == DriverState::Stopped)
        return true;

    if (state_ == DriverState::Created)
    {
        state_ = DriverState::Stopped;
        return true;
    }

    if (state_ == DriverState::Finished)
    {
        state_ = DriverState::Stopped;
        return true;
    }

    if (state_ == DriverState::Error)
    {
        if (player_)
            player_->stop();

        state_ = DriverState::Stopped;
        return true;
    }

    state_ = DriverState::Stopping;

    if (player_)
        player_->stop();

    state_ = DriverState::Stopped;

    return true;
}

bool EurocDatasetDriver::isRunning() const
{
    return state_ == DriverState::Running;
}

DriverState EurocDatasetDriver::getState() const
{
    if (state_ == DriverState::Running &&
        player_)
    {
        if (player_->hasError())
            return DriverState::Error;

        if (player_->isFinished())
            return DriverState::Finished;
    }

    return state_.load();
}