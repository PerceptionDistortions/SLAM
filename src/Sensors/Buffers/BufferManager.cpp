#include "Sensors/Buffers/BufferManager.h"

#include <utility>

bool BufferManager::registerSensor(
    const std::string& sensor_id,
    std::size_t max_size,
    std::int64_t timeout_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (sensor_id.empty())
        return false;

    if (max_size == 0)
        return false;

    if (timeout_ms < 0)
        return false;

    if (buffers_.find(sensor_id) != buffers_.end())
        return false;

    SensorBuffer buffer;

    buffer.max_size = max_size;
    buffer.timeout_ms = timeout_ms;

    buffer.last_received_time =
        Clock::now();

    buffer.active = true;

    buffers_.emplace(
        sensor_id,
        std::move(buffer));

    return true;
}


// ============================================================
// NON-BLOCKING PUSH
// ============================================================

bool BufferManager::push(Data data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string sensor_id =
        getSensorId(data);

    auto it = buffers_.find(sensor_id);

    if (it == buffers_.end())
        return false;

    SensorBuffer& sensor_buffer =
        it->second;

    const std::int64_t timestamp =
        getTimestamp(data);

    // --------------------------------------------------------
    // Per-sensor timestamp monotonicity
    // --------------------------------------------------------

    if (sensor_buffer.has_received_data &&
        timestamp < sensor_buffer.last_timestamp)
    {
        return false;
    }

    // --------------------------------------------------------
    // Bounded buffer
    // --------------------------------------------------------

    if (sensor_buffer.data.size() >=
        sensor_buffer.max_size)
    {
        return false;
    }

    sensor_buffer.data.push_back(
        std::move(data));

    sensor_buffer.last_timestamp =
        timestamp;

    sensor_buffer.has_received_data =
        true;

    sensor_buffer.last_received_time =
        Clock::now();

    // Sensor is producing data again.
    sensor_buffer.active = true;

    cv_not_empty_.notify_one();

    return true;
}


// ============================================================
// BLOCKING PUSH
// ============================================================

bool BufferManager::pushBlocking(Data data,const std::atomic<bool>& running)
{
    std::unique_lock<std::mutex> lock(mutex_);
    const std::string sensor_id = getSensorId(data);
    auto it = buffers_.find(sensor_id);
    if (it == buffers_.end()) return false;
    SensorBuffer& sensor_buffer = it->second;
    const std::int64_t timestamp = getTimestamp(data);

    // --------------------------------------------------------
    // Per-sensor timestamp monotonicity
    // --------------------------------------------------------

    if (sensor_buffer.has_received_data && timestamp < sensor_buffer.last_timestamp)
    {
        return false;
    }

    // --------------------------------------------------------
    // Wait until there is space in the buffer
    // or the producer is being stopped.
    // --------------------------------------------------------

    cv_not_full_.wait(lock,[&]()
        {
            return sensor_buffer.data.size() < sensor_buffer.max_size
                || !running.load();
        });

    if (!running.load()) return false;

    // --------------------------------------------------------
    // Insert measurement
    // --------------------------------------------------------

    sensor_buffer.data.push_back(std::move(data));
    sensor_buffer.last_timestamp =timestamp;
    sensor_buffer.has_received_data =true;
    sensor_buffer.last_received_time =Clock::now();
    sensor_buffer.active = true;
    cv_not_empty_.notify_one();
    return true;
}


// ============================================================
// POP NEXT
// ============================================================

bool BufferManager::popNext(Data& data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Update timeout state before calculating
    // the watermark.
    updateTimeoutStates();

    const std::int64_t watermark =
        calculateWatermark();

    // No valid watermark yet.
    if (watermark ==
        std::numeric_limits<std::int64_t>::min())
    {
        return false;
    }

    std::string selected_sensor;

    std::int64_t earliest_timestamp =
        std::numeric_limits<std::int64_t>::max();

    for (const auto& [sensor_id, sensor_buffer]
         : buffers_)
    {
        if (sensor_buffer.data.empty())
            continue;

        const std::int64_t timestamp =
            getTimestamp(
                sensor_buffer.data.front());

        /*
         * Active sensors:
         *
         * Only release data up to the watermark.
         *
         * Inactive/finished sensors:
         *
         * Their already-buffered data can still be
         * drained.
         */
        if (sensor_buffer.active &&
            timestamp > watermark)
        {
            continue;
        }

        if (timestamp < earliest_timestamp)
        {
            earliest_timestamp =
                timestamp;

            selected_sensor =
                sensor_id;
        }
    }

    if (selected_sensor.empty())
        return false;

    SensorBuffer& selected_buffer =
        buffers_.at(selected_sensor);

    data =std::move(selected_buffer.data.front());
    selected_buffer.data.pop_front();

    // A producer waiting because its buffer was full
    // can now continue.
    cv_not_full_.notify_all();

    return true;
}


// ============================================================
// MARK SENSOR FINISHED
// ============================================================

bool BufferManager::markFinished(
    const std::string& sensor_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it =
        buffers_.find(sensor_id);

    if (it == buffers_.end())
        return false;

    /*
     * The sensor no longer participates in the
     * watermark calculation.
     *
     * Existing buffered data is NOT deleted.
     */
    it->second.active = false;

    cv_not_empty_.notify_all();

    return true;
}


// ============================================================
// ACTIVATE SENSOR
// ============================================================

bool BufferManager::activateSensor(
    const std::string& sensor_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it =
        buffers_.find(sensor_id);

    if (it == buffers_.end())
        return false;

    it->second.active = true;

    it->second.last_received_time =
        Clock::now();

    return true;
}


// ============================================================
// TIMEOUT
// ============================================================

void BufferManager::updateTimeoutStates()
{
    const Clock::time_point now =
        Clock::now();

    for (auto& [sensor_id, sensor_buffer]
         : buffers_)
    {
        // Timeout disabled.
        if (sensor_buffer.timeout_ms == 0)
            continue;

        /*
         * Do not timeout a sensor that has never
         * produced its first measurement.
         *
         * This avoids incorrectly timing out a sensor
         * during startup.
         */
        if (!sensor_buffer.has_received_data)
            continue;

        const auto elapsed =
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                    now -
                    sensor_buffer.last_received_time);

        if (elapsed.count() >=
            sensor_buffer.timeout_ms)
        {
            sensor_buffer.active = false;
        }
    }
}


// ============================================================
// WATERMARK
// ============================================================

std::int64_t BufferManager::calculateWatermark() const
{
    std::int64_t watermark =
        std::numeric_limits<std::int64_t>::max();

    bool found_active_sensor = false;

    for (const auto& [sensor_id, sensor_buffer]
         : buffers_)
    {
        /*
         * Finished or timed-out sensors don't
         * participate in the watermark.
         */
        if (!sensor_buffer.active)
            continue;

        /*
         * An active sensor must have produced at
         * least one measurement before a watermark
         * can be established.
         */
        if (!sensor_buffer.has_received_data)
        {
            return std::numeric_limits<
                std::int64_t>::min();
        }

        watermark =
            std::min(
                watermark,
                sensor_buffer.last_timestamp);

        found_active_sensor = true;
    }

    if (!found_active_sensor)
    {
        return std::numeric_limits<
            std::int64_t>::min();
    }

    return watermark;
}


// ============================================================
// NOTIFY ALL
// ============================================================

void BufferManager::notifyAll()
{
    cv_not_full_.notify_all();
    cv_not_empty_.notify_all();
}


// ============================================================
// GLOBAL EMPTY
// ============================================================

bool BufferManager::empty() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& [sensor_id, sensor_buffer]
         : buffers_)
    {
        if (!sensor_buffer.data.empty())
            return false;
    }

    return true;
}


// ============================================================
// GLOBAL SIZE
// ============================================================

std::size_t BufferManager::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::size_t total = 0;

    for (const auto& [sensor_id, sensor_buffer]
         : buffers_)
    {
        total +=
            sensor_buffer.data.size();
    }

    return total;
}


// ============================================================
// SENSOR EMPTY
// ============================================================

bool BufferManager::empty(
    const std::string& sensor_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it =
        buffers_.find(sensor_id);

    if (it == buffers_.end())
        return true;

    return it->second.data.empty();
}


// ============================================================
// SENSOR SIZE
// ============================================================

std::size_t BufferManager::size(
    const std::string& sensor_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it =
        buffers_.find(sensor_id);

    if (it == buffers_.end())
        return 0;

    return it->second.data.size();
}


// ============================================================
// GET SENSOR ID
// ============================================================

std::string BufferManager::getSensorId(
    const Data& data)
{
    return std::visit(
        [](const auto& sensor_data)
        {
            return sensor_data.sensor_id;
        },
        data);
}


// ============================================================
// GET TIMESTAMP
// ============================================================

std::int64_t BufferManager::getTimestamp(
    const Data& data)
{
    return std::visit(
        [](const auto& sensor_data)
        {
            return sensor_data.timestamp;
        },
        data);
}