
#include "Configuration/NewConfigLoader.h"

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <yaml-cpp/yaml.h>

// =============================================================
// YAML HELPERS
// =============================================================

static YAML::Node loadYamlSection(const std::string& value)
{
    try
    {
        return YAML::Load(value);
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(
            "Failed to parse YAML section: " +
            std::string(e.what()));
    }
}

template <typename T>
static T readOrDefault(
    const YAML::Node& node,
    const std::string& key,
    const T& defaultValue)
{
    const YAML::Node value = node[key];

    if (!value || value.IsNull())
    {
        return defaultValue;
    }

    try
    {
        return value.as<T>();
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(
            "Invalid configuration value for '" + key +
            "': " + e.what());
    }
}

// =============================================================
// LOAD COMPLETE SYSTEM CONFIGURATION
// =============================================================

SystemConfigs NewConfigLoader::load(const std::string& filePath)
{
    YAML::Node root;

    try
    {
        root = YAML::LoadFile(filePath);
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(
            "Failed to load YAML configuration '" + filePath +
            "': " + e.what());
    }

    if (!root.IsMap())
    {
        throw std::runtime_error(
            "Invalid configuration: root must be a YAML mapping.");
    }

    SystemConfigs config;

    // SLAM SYSTEM
    if (root["slam_system"])
    {
        config.slam = parseSLAM(
            YAML::Dump(root["slam_system"]));
    }

    // SENSOR DATA SOURCE
    if (root["sensor_data_source"])
    {
        config.sensorDataSource = parseSensorDataSource(
            YAML::Dump(root["sensor_data_source"]));
    }

    // SENSORS
    if (root["sensors"])
    {
        config.sensors = parseSensors(
            YAML::Dump(root["sensors"]));
    }

    // VISUAL FRONTEND
    if (root["visual_frontend"])
    {
        config.visualFrontend = parseVisualFrontend(
            YAML::Dump(root["visual_frontend"]));
    }

    // TRACK MANAGEMENT
    if (root["track_management"])
    {
        config.visualTracks = parseVisualTracks(
            YAML::Dump(root["track_management"]));
    }

    // PNP
    if (root["pnp"])
    {
        config.pnp = parsePnp(
            YAML::Dump(root["pnp"]));
    }

    // TRIANGULATION
    if (root["triangulation"])
    {
        config.triangulation = parseTriangulation(
            YAML::Dump(root["triangulation"]));
    }

    // LOGGING
    if (root["logging"])
    {
        config.logging = parseLogging(
            YAML::Dump(root["logging"]));
    }

    return config;
}

// =============================================================
// SLAM CONFIGURATION
// =============================================================

SLAMConfigs NewConfigLoader::parseSLAM(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    SLAMConfigs config;

    config.type = readOrDefault(
        node, "type", config.type);

    config.monoCamId = readOrDefault(
        node, "mono_cam_id", config.monoCamId);

    config.monoViCamId = readOrDefault(
        node, "mono_vi_cam_id", config.monoViCamId);

    config.monoViImuId = readOrDefault(
        node, "mono_vi_imu_id", config.monoViImuId);

    config.stereoViLeftCamId = readOrDefault(
        node, "stereo_vi_left_cam_id",
        config.stereoViLeftCamId);

    config.stereoViRightCamId = readOrDefault(
        node, "stereo_vi_right_cam_id",
        config.stereoViRightCamId);

    config.stereoViImuId = readOrDefault(
        node, "stereo_vi_imu_id",
        config.stereoViImuId);

    config.visualizer = readOrDefault(
        node, "visualizer", config.visualizer);

    config.keyBasedControlledPlayback=readOrDefault(
        node,"controlled_playback",config.keyBasedControlledPlayback
    );

    return config;
}

// =============================================================
// SENSOR DATA SOURCE CONFIGURATION
// =============================================================

SensorDataSourceConfig NewConfigLoader::parseSensorDataSource(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    SensorDataSourceConfig config;

    config.type = readOrDefault(
        node, "type", config.type);

    config.datasetId = readOrDefault(
        node, "dataset_id", config.datasetId);

    config.datasetPath = readOrDefault(
        node, "dataset_path", config.datasetPath);

    config.playbackRate = readOrDefault(
        node, "playback_rate", config.playbackRate);

    if (config.playbackRate <= 0.0)
    {
        throw std::runtime_error(
            "playback_rate must be greater than zero.");
    }

    return config;
}

// =============================================================
// SENSOR CONFIGURATION
// =============================================================

std::vector<SensorsConfig> NewConfigLoader::parseSensors(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    if (!node.IsSequence())
    {
        throw std::runtime_error(
            "'sensors' must be a YAML sequence.");
    }

    std::vector<SensorsConfig> sensors;
    std::unordered_set<std::string> sensorIds;

    for (const YAML::Node& sensorNode : node)
    {
        SensorsConfig sensor;

        // BASIC SENSOR INFORMATION
        sensor.id = readOrDefault(
            sensorNode, "id", sensor.id);

        sensor.type = readOrDefault(
            sensorNode, "type", sensor.type);

        sensor.sourceId = readOrDefault(
            sensorNode, "source_id", sensor.sourceId);

        sensor.driver = readOrDefault(
            sensorNode, "driver", sensor.driver);

        if (sensor.id.empty())
        {
            throw std::runtime_error(
                "A sensor configuration has an empty ID.");
        }

        if (!sensorIds.insert(sensor.id).second)
        {
            throw std::runtime_error(
                "Duplicate sensor ID: " + sensor.id);
        }

        // SENSOR-SPECIFIC CONFIGURATION
        const YAML::Node sensorConfig = sensorNode["config"];

        if (sensorConfig)
        {
            // DATASET SETTINGS
            sensor.dataset.stream = readOrDefault(
                sensorConfig, "stream",
                sensor.dataset.stream);

            sensor.dataset.dataPath = readOrDefault(
                sensorConfig, "data_path",
                sensor.dataset.dataPath);

            sensor.dataset.csv = readOrDefault(
                sensorConfig, "csv",
                sensor.dataset.csv);

            // PHYSICAL HARDWARE SETTINGS
            sensor.physical.device = readOrDefault(
                sensorConfig, "device",
                sensor.physical.device);

            sensor.physical.baudRate = readOrDefault(
                sensorConfig, "baud_rate",
                sensor.physical.baudRate);

            
            // CALIBRATION
            const YAML::Node calibrationNode =
                sensorConfig["calibration"];

            if (calibrationNode)
            {
                sensor.calibration.sensorId = sensor.id;
                sensor.calibration.sensorType = sensor.type;

                sensor.calibration.providerType = readOrDefault(
                    calibrationNode,
                    "type",
                    sensor.calibration.providerType);

                sensor.calibration.provider = readOrDefault(
                    calibrationNode,
                    "name",
                    sensor.calibration.provider);

                sensor.calibration.file = readOrDefault(
                    calibrationNode,
                    "file",
                    sensor.calibration.file);
            }

        }

        sensors.push_back(std::move(sensor));
    }

    return sensors;
}

// =============================================================
// VISUAL FRONTEND CONFIGURATION
// =============================================================

VisualFrontendConfigs NewConfigLoader::parseVisualFrontend(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    VisualFrontendConfigs config;

    // PREPROCESSING
    config.motionBlur = readOrDefault(node, "motion_blur", config.motionBlur);
    config.lightExposure = readOrDefault(node, "light_exposure", config.lightExposure);

    // CLAHE PARAMETERS
    config.claheCliplimit = readOrDefault(node, "clahe_cliplimit", config.claheCliplimit);
    config.claheTileLimitSize = readOrDefault(node, "clahe_tile_grid_size", config.claheTileLimitSize);

    //LAPLACIAN
    config.laplacianVarianceThreshold = readOrDefault(
    node,
    "laplacian_variance_threshold",
    config.laplacianVarianceThreshold);

    // FEATURE DETECTION AND MATCHING
    config.detectionDescription = readOrDefault(
        node, "detection_description",
        config.detectionDescription);

    config.featureMatching = readOrDefault(
        node, "feature_matching",
        config.featureMatching);

    config.featureTracking = readOrDefault(
        node, "feature_tracking",
        config.featureTracking);

    // ORB PARAMETERS
    config.orbFeatureCount = readOrDefault(
        node, "orb_feature_count",
        config.orbFeatureCount);

    config.orbScaleFactor = readOrDefault(
        node, "orb_scale_factor",
        config.orbScaleFactor);

    config.orbLevels = readOrDefault(
        node, "orb_levels", config.orbLevels);

    // DESCRIPTOR MATCHING AND FILTERING
    config.loweRatio = readOrDefault(
        node, "lowe_ratio", config.loweRatio);

    config.crossCheck = readOrDefault(
        node, "cross_check", config.crossCheck);

    config.distanceFiltering = readOrDefault(
        node, "distance_filtering",
        config.distanceFiltering);

    //FIXED DISTANCE PARAMS
    config.fixedDistanceFilterMaxDistance=readOrDefault(
        node,"fixed_distance_filter_max_distance",
        config.fixedDistanceFilterMaxDistance
    );

    // MOTION ESTIMATION
    config.recoverPoseDistanceThreshold = readOrDefault(
        node, "recover_pose_distance_threshold",
        config.recoverPoseDistanceThreshold);

    // MOTION GATING
    config.eRansacMatchCount = readOrDefault(
        node, "e_ransac_match_count",
        config.eRansacMatchCount);

    config.pnpRansacCount = readOrDefault(
        node, "pnp_ransac_count",
        config.pnpRansacCount);

    return config;
}

// =============================================================
// VISUAL TRACK CONFIGURATION
// =============================================================

VisualTrackConfig NewConfigLoader::parseVisualTracks(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    VisualTrackConfig config;

    // TENTATIVE TRACKS
    config.tentative_max_age = readOrDefault(
        node, "tentative_age",
        config.tentative_max_age);

    config.tentative_max_misses = readOrDefault(
        node, "tentative_max_miss",
        config.tentative_max_misses);

    config.tentative_max_match = readOrDefault(
        node, "tentative_max_match",
        config.tentative_max_match);

    // ACTIVE TRACKS
    config.active_max_age = readOrDefault(
        node, "active_age", config.active_max_age);

    config.active_max_misses = readOrDefault(
        node, "active_max_miss",
        config.active_max_misses);

    config.active_max_matches = readOrDefault(
        node, "active_max_match",
        config.active_max_matches);

    // TRIANGULATED TRACKS
    config.triangulated_max_observations = readOrDefault(
        node, "triangulated_max_observations",
        config.triangulated_max_observations);

    config.triangulated_max_matches = readOrDefault(
        node, "triangulated_max_matches",
        config.triangulated_max_matches);

    // Accept the corrected key and the previous YAML typo.
    config.triangulated_max_misses = readOrDefault(
        node, "triangulated_max_misses",
        readOrDefault(
            node, "triangultaed_max_miss",
            config.triangulated_max_misses));

    // TRIANGULATED LOST TRACKS
    config.triangulated_lost_max_miss = readOrDefault(
        node, "triangulated_lost_max_miss",
        config.triangulated_lost_max_miss);

    config.triangulated_lost_max_match = readOrDefault(
        node, "triangulated_lost_max_match",
        config.triangulated_lost_max_match);

    // LANDMARKS
    config.landmark_max_observations = readOrDefault(
        node, "landmark_max_observations",
        config.landmark_max_observations);

    config.landmark_max_misses = readOrDefault(
        node, "landmark_max_miss",
        config.landmark_max_misses);

    // LANDMARK LOST TRACKS
    config.landmark_lost_max_misses = readOrDefault(
        node, "landmark_lost_max_miss",
        config.landmark_lost_max_misses);

    config.landmark_lost_max_matches = readOrDefault(
        node, "landmark_lost_max_match",
        config.landmark_lost_max_matches);

    return config;
}

// =============================================================
// PNP CONFIGURATION
// =============================================================

PnpConfig NewConfigLoader::parsePnp(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    PnpConfig config;

    config.pnpStrategy =
        readOrDefault(node, "pnp_strategy", config.pnpStrategy);

    config.minimumLandmarkPnp = readOrDefault(
        node, "minimum_landmark_pnp",
        config.minimumLandmarkPnp);

    config.pnpIterationsCount = readOrDefault(
        node, "pnp_iterations_count",
        config.pnpIterationsCount);

    config.pnpReprojectionThreshold = readOrDefault(
        node, "pnp_reprojection_threshold",
        config.pnpReprojectionThreshold);

    config.pnpConfidence = readOrDefault(
        node, "pnp_confidence",
        config.pnpConfidence);

    config.pnpMinCorrespondence = readOrDefault(
        node, "pnp_min_correspondence",
        config.pnpMinCorrespondence);

    config.pnpRefineWithInliers = readOrDefault(
        node, "pnp_refine_with_inliers",
        config.pnpRefineWithInliers);

    return config;
}

// =============================================================
// TRIANGULATION CONFIGURATION
// =============================================================

TriangulationConfig NewConfigLoader::parseTriangulation(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    TriangulationConfig config;

    config.triangulator = readOrDefault(
        node, "triangulator", config.triangulator);

    return config;
}

// =============================================================
// LOGGING CONFIGURATION
// =============================================================

LoggerConfig NewConfigLoader::parseLogging(
    const std::string& value)
{
    const YAML::Node node = loadYamlSection(value);

    LoggerConfig config;

    config.logSensors = readOrDefault(
        node, "sensors", config.logSensors);

    config.logVisualFrontend = readOrDefault(
        node, "visual_frontend",
        config.logVisualFrontend);

    config.logEstimation = readOrDefault(
        node, "estimation", config.logEstimation);

    config.logVisualization = readOrDefault(
        node, "visualization",
        config.logVisualization);

    config.logMap = readOrDefault(
        node, "map", config.logMap);

    return config;
}
