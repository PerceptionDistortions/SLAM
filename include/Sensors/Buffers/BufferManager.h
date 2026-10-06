//THIS CLASS WILL HOLD BUFFERS OF ALL SENSORS
//WILL BE OWNED BY SENSOR MANAGER
//SENSOR MANAGER WILL FETCH SENSOR BUFFERS BASED ON SENSOR IDs
//WILL DELIVER SENSOR DATA CHRONOLOGICALLY BASED ON TIMESTAMP
//WILL ENABLE BUFFERS ONLY FOR THE SENSORS BEING USED
//FOR ASYN SENSORS: WATERMARK, BOUNDED BUFFER, TIMEOUT POLICY

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <variant>

#include "Sensors/SensorDataClasses/CameraData.h"
#include "Sensors/SensorDataClasses/ImuData.h"

class BufferManager
{
public:

    using Data = std::variant<
        CameraData,
        ImuData
    >;

    using Buffer = std::deque<Data>;
    using Clock = std::chrono::steady_clock;

    // timeout_ms = 0 disables timeout handling.
    bool registerSensor(
        const std::string& sensor_id,
        std::size_t max_size,
        std::int64_t timeout_ms = 0);

    // Non-blocking push.
    // Returns false if the buffer is full or the
    // timestamp is invalid.
    bool push(Data data);

    // Blocking push used by producer threads.
    // Waits while the sensor buffer is full.
    bool pushBlocking(
        Data data,
        const std::atomic<bool>& running);

    // Remove the earliest measurement that is safe
    // according to the watermark.
    //Store next data in data
    bool popNext(Data& data);

    // Mark a sensor as finished.
    // Finished sensors no longer block the watermark.
    bool markFinished(const std::string& sensor_id);

    // Reactivate a sensor.
    bool activateSensor(const std::string& sensor_id);

    // Wake all waiting producer/consumer threads.
    void notifyAll();

    bool empty() const;

    std::size_t size() const;

    bool empty(
        const std::string& sensor_id) const;

    std::size_t size(
        const std::string& sensor_id) const;

private:

    struct SensorBuffer
    {
        // Sensor measurements.
        Buffer data;

        // Maximum number of measurements.
        std::size_t max_size = 0;

        // Timestamp validation.
        bool has_received_data = false;
        std::int64_t last_timestamp = 0;

        // Timeout configuration.
        std::int64_t timeout_ms = 0;
        Clock::time_point last_received_time{};

        // Participates in watermark calculation.
        bool active = true;
    };

private:

    static std::string getSensorId(
        const Data& data);

    static std::int64_t getTimestamp(
        const Data& data);

    // Marks sensors that have not produced data within
    // their configured timeout as inactive.
    void updateTimeoutStates();

    // Calculates the timestamp up to which data from
    // all active sensors is known to have arrived.
    //
    // Returns INT64_MIN if the watermark cannot yet
    // be established.
    std::int64_t calculateWatermark() const;

private:

    mutable std::mutex mutex_;

    std::condition_variable cv_not_full_;
    std::condition_variable cv_not_empty_;

    std::unordered_map<
        std::string,
        SensorBuffer
    > buffers_;
};