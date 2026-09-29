#include "Sensors/SensorManager/SensorManager.h"

bool SensorManager::addSensor(std::unique_ptr<SensorContext> sensor)
{
    if (!sensor) return false;
    const std::string& name = sensor->getName();

    // Don't allow duplicate sensor names
    if (sensors_.find(name) != sensors_.end()) return false;

    //Create a buffer for this sensor in the buffer manager
    if (!buffer_manager_.registerSensor(name, 1000, 0)) return false;
    sensors_.emplace(name,std::move(sensor));
    return true;
}

bool SensorManager::init()
{
    for (auto& [name, sensor] : sensors_)
    {
        if (!sensor->init()){
            std::cerr << "Failed to initialize sensor: " << name << std::endl;
            return false;
        }
    }

    return true;
}

bool SensorManager::start()
{
    for (auto& [name, sensor] : sensors_)
    {
        if (!sensor->start()){
            std::cerr << "Failed to start sensor: " << name << std::endl;
            return false;
        }
    }

    return true;
}

bool SensorManager::stop()
{
    bool success = true;

    for (auto& [name, sensor] : sensors_)
    {
        if (!sensor->stop()) success = false;
    }
    return success;
}

std::shared_ptr<SensorContext>
SensorManager::getSensor(const std::string& name)
{
    auto it = sensors_.find(name);
    if (it == sensors_.end()) return nullptr;
    return it->second;
}

bool SensorManager::loadConfig(const std::string& config_path)
{
    try
    {
        //LOAD CONFIG FROM YAML
        //CHECK IF NO CONFIG EXISTS
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config["data_sources"])
        {
            std::cerr<< "Error: data_sources configuration missing\n";
            return false;
        }

        const YAML::Node data_sources =config["data_sources"];
        if (!data_sources.IsSequence())
        {
            std::cerr << "Error: data_sources must be a sequence\n";
            return false;
        }

        // Find the EuroC data source
        YAML::Node euroc_source;

        for (const auto& source : data_sources)
        {
            YAML::Node source_node = source;

            if (source_node["id"] &&
                source_node["id"].as<std::string>() == "euroc")
            {
                euroc_source = source_node;
                break;
            }
        }

        if (!euroc_source)
        {
            std::cerr << "Error: euroc data source missing\n";
            return false;
        }

        if (!euroc_source["dataset"])
        {
            std::cerr << "Error: dataset configuration missing\n";
            return false;
        }

        const YAML::Node dataset =
            euroc_source["dataset"];

        // ============================================================
        // DATASET NAME
        // ============================================================

        if (!dataset["name"])
        {
            std::cerr
                << "Error: dataset name missing\n";

            return false;
        }

        const std::string dataset_name =
            dataset["name"].as<std::string>();


        // ============================================================
        // CURRENTLY SUPPORT EUROC DATASET
        // ============================================================

        if (dataset_name != "euroc")
        {
            std::cerr
                << "Unsupported dataset: "
                << dataset_name
                << '\n';

            return false;
        }


        // ============================================================
        // DATASET PATH
        // ============================================================

        if (!dataset["path"])
        {
            std::cerr
                << "Error: dataset path missing\n";

            return false;
        }

        const std::string dataset_path =
            dataset["path"].as<std::string>();


        // ============================================================
        // SENSOR CONFIGURATION
        // ============================================================

        if (!config["sensors"] ||
            !config["sensors"].IsSequence())
        {
            std::cerr
                << "Error: sensors configuration missing\n";

            return false;
        }


        // ============================================================
        // CREATE EACH CONFIGURED SENSOR
        // ============================================================

        for (const auto& sensor_node : config["sensors"])
        {
            // ========================================================
            // SENSOR ID
            // ========================================================

            if (!sensor_node["id"])
            {
                std::cerr
                    << "Error: sensor id missing\n";

                return false;
            }

            const std::string id =
                sensor_node["id"].as<std::string>();


            // ========================================================
            // SENSOR TYPE
            // ========================================================

            if (!sensor_node["type"])
            {
                std::cerr
                    << "Error: sensor type missing for sensor: "
                    << id << '\n';

                return false;
            }

            const std::string type =
                sensor_node["type"].as<std::string>();


            // ========================================================
            // DATA SOURCE
            // ========================================================

            if (!sensor_node["source_id"])
            {
                std::cerr
                    << "Error: sensor source missing for sensor: "
                    << id << '\n';

                return false;
            }

            const std::string source =
                sensor_node["source_id"].as<std::string>();


            // ========================================================
            // DRIVER
            // ========================================================

            if (!sensor_node["driver"])
            {
                std::cerr
                    << "Error: sensor driver missing for sensor: "
                    << id << '\n';

                return false;
            }

            const std::string driver =
                sensor_node["driver"].as<std::string>();


            // ========================================================
            // VALIDATE SOURCE / DRIVER
            // ========================================================

            if (source != "euroc" ||
                driver != "euroc")
            {
                std::cerr
                    << "Unsupported source/driver for sensor: "
                    << id << '\n';

                return false;
            }


            // ========================================================
            // SENSOR-SPECIFIC CONFIGURATION
            // ========================================================

            if (!sensor_node["config"])
            {
                std::cerr
                    << "Error: sensor config missing for sensor: "
                    << id << '\n';

                return false;
            }

            const YAML::Node sensor_config =
                sensor_node["config"];


            // ========================================================
            // STREAM
            // ========================================================

            if (!sensor_config["stream"])
            {
                std::cerr
                    << "Error: stream missing for sensor: "
                    << id << '\n';

                return false;
            }

            const std::string stream =
                sensor_config["stream"].as<std::string>();


            // ========================================================
            // CSV
            // ========================================================

            if (!sensor_config["csv"])
            {
                std::cerr
                    << "Error: csv missing for sensor: "
                    << id << '\n';

                return false;
            }

            const std::string csv =
                sensor_config["csv"].as<std::string>();


            // ========================================================
            // OPTIONAL DATA PATH
            // ========================================================

            std::string data_path;

            if (sensor_config["data_path"])
            {
                data_path =
                    sensor_config["data_path"].as<std::string>();
            }


            // ========================================================
            // CALIBRATION CONFIGURATION
            // ========================================================

            if (!sensor_config["calibration"])
            {
                std::cerr
                    << "Error: calibration configuration missing "
                    << "for sensor: "
                    << id << '\n';

                return false;
            }

            const YAML::Node calibration =
                sensor_config["calibration"];


            // --------------------------------------------------------
            // Validate calibration configuration
            // --------------------------------------------------------

            if (!calibration["type"] ||
                !calibration["name"] ||
                !calibration["file"])
            {
                std::cerr
                    << "Error: incomplete calibration configuration "
                    << "for sensor: "
                    << id << '\n';

                return false;
            }


            // --------------------------------------------------------
            // Build CalibrationConfig
            // --------------------------------------------------------

            CalibrationConfig calibration_config;

            calibration_config.sensorId =
                id;

            calibration_config.sensorType =
                type;

            calibration_config.providerType =
                calibration["type"].as<std::string>();

            calibration_config.provider =
                calibration["name"].as<std::string>();


            // Calibration file is relative to dataset path
            calibration_config.file =
                dataset_path + "/" +
                calibration["file"].as<std::string>();


            // --------------------------------------------------------
            // Load calibration using CalibrationManager
            // --------------------------------------------------------

            std::unique_ptr<ICalibration> calibration_data;

            try
            {
                calibration_data =
                    calibration_manager_.loadCalibration(
                        calibration_config);
            }
            catch (const std::exception& e)
            {
                std::cerr
                    << "Failed to load calibration for sensor: "
                    << id
                    << " : "
                    << e.what()
                    << '\n';

                return false;
            }


            // --------------------------------------------------------
            // Validate returned calibration
            // --------------------------------------------------------

            if (!calibration_data)
            {
                std::cerr
                    << "Failed to create calibration for sensor: "
                    << id
                    << '\n';

                return false;
            }


            // ========================================================
            // CREATE EUROC DATASET DRIVER
            // ========================================================

            PlaybackConfig playback_config;

            if (euroc_source["playback"])
            {
                if (euroc_source["playback"]["realtime"])
                {
                    playback_config.realtime =
                        euroc_source["playback"]["realtime"].as<bool>();
                }

                if (euroc_source["playback"]["rate"])
                {
                    playback_config.rate =
                        euroc_source["playback"]["rate"].as<double>();
                }
            }

            auto driver_object =
                std::make_unique<EurocDatasetDriver>(
                    dataset_path,
                    id,
                    type,
                    data_path,
                    csv,
                    buffer_manager_,
                    playback_config);

            // ========================================================
            // CREATE COMMON SENSOR CONTEXT
            // ========================================================

            auto sensor =
                std::make_unique<SensorContext>(
                    id,
                    std::move(driver_object));


            // ========================================================
            // ATTACH CALIBRATION TO SENSOR CONTEXT
            // ========================================================

            sensor->setCalibration(std::move(calibration_data));


            // ========================================================
            // REGISTER SENSOR
            // ========================================================

            // addSensor() handles:
            //
            // 1. SensorContext registration
            // 2. Buffer registration

            if (!addSensor(std::move(sensor)))
            {
                std::cerr
                    << "Failed to add sensor: "
                    << id << '\n';

                return false;
            }


            // ========================================================
            // CLASSIFY SENSOR BY TYPE
            // ========================================================

            if (type == "camera")
            {
                cameras_.push_back(id);
            }
            else if (type == "imu")
            {
                imus_.push_back(id);
            }
            else if (type == "lidar")
            {
                lidars_.push_back(id);
            }
            else
            {
                std::cerr
                    << "Unsupported sensor type: "
                    << type
                    << " for sensor: "
                    << id << '\n';

                return false;
            }


            // ========================================================
            // LOG SUCCESS
            // ========================================================

            std::cout
                << "Added sensor: "
                << id
                << " (" << type << ")"
                << '\n';

            std::cout
                << "Calibration loaded for sensor: "
                << id
                << " | provider="
                << calibration_config.provider
                << " | file="
                << calibration_config.file
                << '\n';
        }


        // ============================================================
        // CONFIGURATION LOADED SUCCESSFULLY
        // ============================================================

        return true;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr
            << "YAML error: "
            << e.what()
            << '\n';

        return false;
    }
}

bool SensorManager::popNext(Data& data)
{
    return buffer_manager_.popNext(data);
}

bool SensorManager::isRunning() const
{
    for (const auto& [name, sensor] : sensors_)
    {
        if (sensor->getDriver().isRunning())
            return true;
    }

    return false;
}