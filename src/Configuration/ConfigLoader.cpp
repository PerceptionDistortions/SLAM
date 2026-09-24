#include"Configuration/ConfigLoader.h"
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <utility>

SystemConfig ConfigLoader::load(
    const std::string& filePath)
{
    YAML::Node root;

    try
    {
        root = YAML::LoadFile(filePath);
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(
            "Failed to load YAML configuration '" +
            filePath +
            "': " +
            e.what());
    }

    SystemConfig config;


    // ============================================================
    // SYSTEM
    // ============================================================

    if (auto node = root["system"])
    {
        if (node["mode"])
        {
            config.system.mode =
                parseSLAMType(
                    node["mode"].as<std::string>());
        }

        if (node["visualizer"])
        {
            config.system.visualizer =
                parseVisualizerType(
                    node["visualizer"].as<std::string>());
        }
    }


    // ============================================================
    // DATA SOURCES
    // ============================================================

    if (auto node = root["data_sources"])
    {
        for (const auto& sourceNode : node)
        {
            DataSourceConfig source;

            if (sourceNode["id"])
                source.id =
                    sourceNode["id"].as<std::string>();

            if (sourceNode["type"])
            {
                source.type =
                    parseDataSourceType(
                        sourceNode["type"].as<std::string>());
            }


            // ----------------------------------------------------
            // Dataset
            // ----------------------------------------------------

            if (auto dataset = sourceNode["dataset"])
            {
                if (dataset["name"])
                    source.dataset.name =
                        dataset["name"].as<std::string>();

                if (dataset["path"])
                    source.dataset.path =
                        dataset["path"].as<std::string>();
            }


            // ----------------------------------------------------
            // Playback
            // ----------------------------------------------------

            if (auto playback = sourceNode["playback"])
            {
                if (playback["realtime"])
                    source.dataset.playback.realtime =
                        playback["realtime"].as<bool>();

                if (playback["rate"])
                    source.dataset.playback.rate =
                        playback["rate"].as<double>();
            }


            // ----------------------------------------------------
            // Physical source
            // ----------------------------------------------------

            if (auto physical = sourceNode["physical"])
            {
                if (physical["transport"])
                    source.physical.transport =
                        physical["transport"].as<std::string>();
            }


            config.dataSources.push_back(
                std::move(source));
        }
    }


    // ============================================================
    // SENSORS
    // ============================================================

    if (auto node = root["sensors"])
    {
        for (const auto& sensorNode : node)
        {
            SensorConfig sensor;

            if (sensorNode["id"])
                sensor.id =
                    sensorNode["id"].as<std::string>();

            if (sensorNode["type"])
                sensor.type =
                    sensorNode["type"].as<std::string>();

            if (sensorNode["source_id"])
                sensor.sourceId =
                    sensorNode["source_id"].as<std::string>();

            if (sensorNode["driver"])
                sensor.driver =
                    sensorNode["driver"].as<std::string>();


            // ====================================================
            // SENSOR CONFIG
            //
            // YAML:
            //
            // config:
            //   stream:
            //   data_path:
            //   csv:
            //   calibration:
            //
            // ====================================================

            if (auto sensorConfig = sensorNode["config"])
            {
                // ----------------------------------------------
                // Dataset
                // ----------------------------------------------

                if (sensorConfig["stream"])
                    sensor.dataset.stream =
                        sensorConfig["stream"].as<std::string>();

                if (sensorConfig["data_path"])
                    sensor.dataset.dataPath =
                        sensorConfig["data_path"].as<std::string>();

                if (sensorConfig["csv"])
                    sensor.dataset.csv =
                        sensorConfig["csv"].as<std::string>();


                // ----------------------------------------------
                // Calibration
                // ----------------------------------------------

                if (auto calibration =
                        sensorConfig["calibration"])
                {
                    if (calibration["type"])
                        sensor.calibration.type =
                            calibration["type"].as<std::string>();

                    if (calibration["name"])
                        sensor.calibration.name =
                            calibration["name"].as<std::string>();

                    if (calibration["file"])
                        sensor.calibration.file =
                            calibration["file"].as<std::string>();
                }


                // ----------------------------------------------
                // Physical sensor
                // ----------------------------------------------

                if (auto physical =
                        sensorConfig["physical"])
                {
                    if (physical["device"])
                        sensor.physical.device =
                            physical["device"].as<std::string>();

                    if (physical["baud_rate"])
                        sensor.physical.baudRate =
                            physical["baud_rate"].as<int>();
                }
            }

            config.sensors.push_back(
                std::move(sensor));
        }
    }


    // ============================================================
    // FRONTEND
    // ============================================================

    if (auto node = root["frontend"])
    {
        // ========================================================
        // VISUAL
        // ========================================================

        if (auto visual = node["visual"])
        {
            if (visual["enabled"])
                config.frontend.visual.enabled =
                    visual["enabled"].as<bool>();


            // ----------------------------------------------------
            // Detector / Descriptor
            // ----------------------------------------------------

            if (auto detector =
                    visual["detector_descriptor"])
            {
                if (detector["type"])
                    config.frontend.visual
                        .detectorDescriptor.type =
                        detector["type"].as<std::string>();

                if (detector["n_features"])
                    config.frontend.visual
                        .detectorDescriptor.nFeatures =
                        detector["n_features"].as<int>();

                if (detector["scale_factor"])
                    config.frontend.visual
                        .detectorDescriptor.scaleFactor =
                        detector["scale_factor"].as<double>();

                if (detector["n_levels"])
                    config.frontend.visual
                        .detectorDescriptor.nLevels =
                        detector["n_levels"].as<int>();
            }


            // ----------------------------------------------------
            // Preprocessing
            // ----------------------------------------------------

            if (auto preprocessing =
                    visual["preprocessing"])
            {
                if (preprocessing["grayscale"])
                    config.frontend.visual
                        .preprocessing.grayscale =
                        preprocessing["grayscale"].as<bool>();

                if (preprocessing["undistort"])
                    config.frontend.visual
                        .preprocessing.undistort =
                        preprocessing["undistort"].as<bool>();
            }


            // ----------------------------------------------------
            // Feature Matching
            // ----------------------------------------------------

            if (auto matching =
                    visual["feature_matching"])
            {
                if (matching["type"])
                    config.frontend.visual
                        .featureMatching.type =
                        matching["type"].as<std::string>();

                if (matching["descriptor"])
                    config.frontend.visual
                        .featureMatching.descriptor =
                        matching["descriptor"].as<std::string>();

                if (matching["ratio_test"])
                    config.frontend.visual
                        .featureMatching.ratioTest =
                        matching["ratio_test"].as<double>();

                if (matching["cross_check"])
                    config.frontend.visual
                        .featureMatching.crossCheck =
                        matching["cross_check"].as<bool>();
            }


            // ----------------------------------------------------
            // Geometric Verification
            // ----------------------------------------------------

            if (auto geometric =
                    visual["geometric_verification"])
            {
                if (geometric["enabled"])
                    config.frontend.visual
                        .geometricVerification.enabled =
                        geometric["enabled"].as<bool>();

                if (geometric["method"])
                    config.frontend.visual
                        .geometricVerification.method =
                        geometric["method"].as<std::string>();

                if (geometric["max_reprojection_error"])
                    config.frontend.visual
                        .geometricVerification
                        .maxReprojectionError =
                        geometric["max_reprojection_error"]
                            .as<double>();
            }


            // ----------------------------------------------------
            // Motion Estimation
            // ----------------------------------------------------

            if (auto motion =
                    visual["motion_estimation"])
            {
                if (motion["enabled"])
                    config.frontend.visual
                        .motionEstimation.enabled =
                        motion["enabled"].as<bool>();
            }


            // ----------------------------------------------------
            // Triangulation
            // ----------------------------------------------------

            if (auto triangulation =
                    visual["triangulation"])
            {
                if (triangulation["enabled"])
                    config.frontend.visual
                        .triangulation.enabled =
                        triangulation["enabled"].as<bool>();

                if (triangulation["min_parallax"])
                    config.frontend.visual
                        .triangulation.minParallax =
                        triangulation["min_parallax"]
                            .as<double>();

                if (triangulation["max_reprojection_error"])
                    config.frontend.visual
                        .triangulation.maxReprojectionError =
                        triangulation["max_reprojection_error"]
                            .as<double>();
            }


            // ----------------------------------------------------
            // Tracking
            // ----------------------------------------------------

            if (auto tracking =
                    visual["tracking"])
            {
                if (tracking["enabled"])
                    config.frontend.visual
                        .tracking.enabled =
                        tracking["enabled"].as<bool>();

                if (tracking["min_tracked_features"])
                    config.frontend.visual
                        .tracking.minTrackedFeatures =
                        tracking["min_tracked_features"]
                            .as<int>();
            }


            // ----------------------------------------------------
            // Landmark Management
            // ----------------------------------------------------

            if (auto landmarks =
                    visual["landmark_management"])
            {
                if (landmarks["enabled"])
                    config.frontend.visual
                        .landmarkManagement.enabled =
                        landmarks["enabled"].as<bool>();
            }


            // ----------------------------------------------------
            // Keyframe Selection
            // ----------------------------------------------------

            if (auto keyframes =
                    visual["keyframe_selection"])
            {
                if (keyframes["enabled"])
                    config.frontend.visual
                        .keyframeSelection.enabled =
                        keyframes["enabled"].as<bool>();
            }
        }


        // ========================================================
        // IMU
        // ========================================================

        if (auto imu = node["imu"])
        {
            if (imu["enabled"])
                config.frontend.imu.enabled =
                    imu["enabled"].as<bool>();

            if (auto preprocessing =
                    imu["preprocessing"])
            {
                if (preprocessing["remove_bias"])
                    config.frontend.imu.preprocessing.removeBias =
                        preprocessing["remove_bias"].as<bool>();

                if (preprocessing["noise_model"])
                    config.frontend.imu.preprocessing.noiseModel =
                        preprocessing["noise_model"]
                            .as<std::string>();
            }

            if (auto preintegration =
                    imu["preintegration"])
            {
                if (preintegration["enabled"])
                    config.frontend.imu.preintegration.enabled =
                        preintegration["enabled"].as<bool>();
            }
        }


        // ========================================================
        // LIDAR
        // ========================================================

        if (auto lidar = node["lidar"])
        {
            if (lidar["enabled"])
                config.frontend.lidar.enabled =
                    lidar["enabled"].as<bool>();
        }
    }


    // ============================================================
    // ESTIMATOR
    // ============================================================

    if (auto node = root["estimator"])
    {
        if (node["type"])
        {
            config.estimator.type =
                parseEstimatorType(
                    node["type"].as<std::string>());
        }


        // ========================================================
        // FILTER
        // ========================================================

        if (auto filter = node["filter"])
        {
            if (filter["type"])
                config.estimator.filter.type =
                    filter["type"].as<std::string>();


            // ----------------------------------------------------
            // Prediction
            // ----------------------------------------------------

            if (auto prediction =
                    filter["prediction"])
            {
                if (prediction["type"])
                {
                    config.estimator.filter.prediction.type =
                        parsePredictorType(
                            prediction["type"]
                                .as<std::string>());
                }


                if (auto cv =
                        prediction["constant_velocity"])
                {
                    if (cv["enabled"])
                        config.estimator.filter
                            .prediction.constantVelocity.enabled =
                            cv["enabled"].as<bool>();
                }


                if (auto imu =
                        prediction["imu"])
                {
                    if (imu["enabled"])
                        config.estimator.filter
                            .prediction.imu.enabled =
                            imu["enabled"].as<bool>();
                }
            }


            // ----------------------------------------------------
            // Correction
            // ----------------------------------------------------

            if (auto correction =
                    filter["correction"])
            {
                if (auto visual =
                        correction["visual"])
                {
                    if (visual["enabled"])
                        config.estimator.filter
                            .correction.visual.enabled =
                            visual["enabled"].as<bool>();

                    if (visual["type"])
                        config.estimator.filter
                            .correction.visual.type =
                            visual["type"].as<std::string>();
                }
            }
        }


        // ========================================================
        // OPTIMIZATION
        // ========================================================

        if (auto optimization =
                node["optimization"])
        {
            if (optimization["enabled"])
                config.estimator.optimization.enabled =
                    optimization["enabled"].as<bool>();

            if (optimization["type"])
                config.estimator.optimization.type =
                    optimization["type"].as<std::string>();

            if (optimization["window_size"])
                config.estimator.optimization.windowSize =
                    optimization["window_size"].as<int>();


            // ----------------------------------------------------
            // Factors
            // ----------------------------------------------------

            if (auto factors =
                    optimization["factors"])
            {
                if (factors["visual"])
                    config.estimator.optimization.factors.visual =
                        factors["visual"].as<bool>();

                if (factors["imu"])
                    config.estimator.optimization.factors.imu =
                        factors["imu"].as<bool>();
            }


            // ----------------------------------------------------
            // Marginalization
            // ----------------------------------------------------

            if (auto marginalization =
                    optimization["marginalization"])
            {
                if (marginalization["enabled"])
                    config.estimator.optimization
                        .marginalization.enabled =
                        marginalization["enabled"].as<bool>();
            }


            // ----------------------------------------------------
            // Solver
            // ----------------------------------------------------

            if (auto solver =
                    optimization["solver"])
            {
                if (solver["type"])
                    config.estimator.optimization.solver.type =
                        parseOptimizerType(
                            solver["type"].as<std::string>());

                if (solver["algorithm"])
                    config.estimator.optimization.solver.algorithm =
                        solver["algorithm"].as<std::string>();

                if (solver["max_iterations"])
                    config.estimator.optimization.solver.maxIterations =
                        solver["max_iterations"].as<int>();
            }
        }
    }


    // ============================================================
    // BACKEND
    // ============================================================

    if (auto node = root["backend"])
    {
        if (node["enabled"])
            config.backend.enabled =
                node["enabled"].as<bool>();


        // ========================================================
        // LOCAL
        // ========================================================

        if (auto local = node["local"])
        {
            config.backend.local =
                parseBackendOptimization(local);
        }


        // ========================================================
        // GLOBAL
        // ========================================================

        if (auto global = node["global"])
        {
            config.backend.global =
                parseBackendOptimization(global);
        }


        // ========================================================
        // POSE GRAPH
        // ========================================================

        if (auto poseGraph =
                node["pose_graph"])
        {
            if (poseGraph["enabled"])
                config.backend.poseGraph.enabled =
                    poseGraph["enabled"].as<bool>();


            if (auto optimize =
                    poseGraph["optimize"])
            {
                if (optimize["poses"])
                    config.backend.poseGraph.optimize.poses =
                        optimize["poses"].as<bool>();
            }


            if (auto solver =
                    poseGraph["solver"])
            {
                if (solver["type"])
                    config.backend.poseGraph.solver.type =
                        parseOptimizerType(
                            solver["type"].as<std::string>());

                if (solver["algorithm"])
                    config.backend.poseGraph.solver.algorithm =
                        solver["algorithm"].as<std::string>();

                if (solver["max_iterations"])
                    config.backend.poseGraph.solver.maxIterations =
                        solver["max_iterations"].as<int>();
            }
        }
    }


    // ============================================================
    // LOOP CLOSURE
    // ============================================================

    if (auto node = root["loop_closure"])
    {
        if (node["enabled"])
            config.loopClosure.enabled =
                node["enabled"].as<bool>();


        // --------------------------------------------------------
        // Place Recognition
        // --------------------------------------------------------

        if (auto place =
                node["place_recognition"])
        {
            if (place["type"])
                config.loopClosure.placeRecognition.type =
                    place["type"].as<std::string>();

            if (auto vocabulary =
                    place["vocabulary"])
            {
                if (vocabulary["file"])
                    config.loopClosure.placeRecognition
                        .vocabulary.file =
                        vocabulary["file"].as<std::string>();
            }

            if (place["similarity_threshold"])
                config.loopClosure.placeRecognition
                    .similarityThreshold =
                    place["similarity_threshold"].as<double>();

            if (place["min_temporal_distance"])
                config.loopClosure.placeRecognition
                    .minTemporalDistance =
                    place["min_temporal_distance"].as<int>();
        }


        // --------------------------------------------------------
        // Candidate Verification
        // --------------------------------------------------------

        if (auto candidate =
                node["candidate_verification"])
        {
            if (candidate["enabled"])
                config.loopClosure.candidateVerification.enabled =
                    candidate["enabled"].as<bool>();


            if (auto matching =
                    candidate["feature_matching"])
            {
                if (matching["enabled"])
                    config.loopClosure
                        .candidateVerification
                        .featureMatching.enabled =
                        matching["enabled"].as<bool>();
            }


            if (auto geometric =
                    candidate["geometric_verification"])
            {
                if (geometric["enabled"])
                    config.loopClosure
                        .candidateVerification
                        .geometricVerification.enabled =
                        geometric["enabled"].as<bool>();

                if (geometric["method"])
                    config.loopClosure
                        .candidateVerification
                        .geometricVerification.method =
                        geometric["method"].as<std::string>();

                if (geometric["min_inliers"])
                    config.loopClosure
                        .candidateVerification
                        .geometricVerification.minInliers =
                        geometric["min_inliers"].as<int>();

                if (geometric["max_reprojection_error"])
                    config.loopClosure
                        .candidateVerification
                        .geometricVerification
                        .maxReprojectionError =
                        geometric["max_reprojection_error"]
                            .as<double>();
            }
        }


        // --------------------------------------------------------
        // Constraint
        // --------------------------------------------------------

        if (auto constraint =
                node["constraint"])
        {
            if (constraint["enabled"])
                config.loopClosure.constraint.enabled =
                    constraint["enabled"].as<bool>();

            if (constraint["type"])
                config.loopClosure.constraint.type =
                    constraint["type"].as<std::string>();

            if (constraint["min_inliers"])
                config.loopClosure.constraint.minInliers =
                    constraint["min_inliers"].as<int>();

            if (constraint["max_reprojection_error"])
                config.loopClosure.constraint.maxReprojectionError =
                    constraint["max_reprojection_error"]
                        .as<double>();
        }


        // --------------------------------------------------------
        // Confirmation
        // --------------------------------------------------------

        if (auto confirmation =
                node["confirmation"])
        {
            if (confirmation["enabled"])
                config.loopClosure.confirmation.enabled =
                    confirmation["enabled"].as<bool>();

            if (confirmation["min_consistent_matches"])
                config.loopClosure.confirmation
                    .minConsistentMatches =
                    confirmation["min_consistent_matches"]
                        .as<int>();
        }
    }


    // ============================================================
    // MAP
    // ============================================================

    if (auto node = root["map"])
    {
        if (auto landmarks = node["landmarks"])
        {
            if (landmarks["enabled"])
                config.map.landmarks.enabled =
                    landmarks["enabled"].as<bool>();
        }

        if (auto keyframes = node["keyframes"])
        {
            if (keyframes["enabled"])
                config.map.keyframes.enabled =
                    keyframes["enabled"].as<bool>();
        }

        if (auto localMap = node["local_map"])
        {
            if (localMap["enabled"])
                config.map.localMap.enabled =
                    localMap["enabled"].as<bool>();
        }

        if (auto globalMap = node["global_map"])
        {
            if (globalMap["enabled"])
                config.map.globalMap.enabled =
                    globalMap["enabled"].as<bool>();
        }
    }


    // ============================================================
    // OUTPUT
    // ============================================================

    if (auto node = root["output"])
    {
        if (auto trajectory = node["trajectory"])
        {
            if (trajectory["enabled"])
                config.output.trajectory.enabled =
                    trajectory["enabled"].as<bool>();

            if (trajectory["format"])
                config.output.trajectory.format =
                    trajectory["format"].as<std::string>();

            if (trajectory["path"])
                config.output.trajectory.path =
                    trajectory["path"].as<std::string>();
        }


        if (auto landmarks = node["landmarks"])
        {
            if (landmarks["enabled"])
                config.output.landmarks.enabled =
                    landmarks["enabled"].as<bool>();

            if (landmarks["path"])
                config.output.landmarks.path =
                    landmarks["path"].as<std::string>();
        }


        if (auto metrics = node["metrics"])
        {
            if (metrics["enabled"])
                config.output.metrics.enabled =
                    metrics["enabled"].as<bool>();

            if (metrics["path"])
                config.output.metrics.path =
                    metrics["path"].as<std::string>();
        }


        if (auto covariance = node["covariance"])
        {
            if (covariance["enabled"])
                config.output.covariance.enabled =
                    covariance["enabled"].as<bool>();
        }
    }


    // ============================================================
    // EVALUATION
    // ============================================================

    if (auto node = root["evaluation"])
    {
        if (node["enabled"])
            config.evaluation.enabled =
                node["enabled"].as<bool>();


        if (auto metrics = node["metrics"])
        {
            if (auto ate = metrics["ate"])
            {
                if (ate["enabled"])
                    config.evaluation.metrics.ate.enabled =
                        ate["enabled"].as<bool>();
            }

            if (auto rpe = metrics["rpe"])
            {
                if (rpe["enabled"])
                    config.evaluation.metrics.rpe.enabled =
                        rpe["enabled"].as<bool>();
            }
        }


        if (auto groundTruth =
                node["ground_truth"])
        {
            if (groundTruth["enabled"])
                config.evaluation.groundTruth.enabled =
                    groundTruth["enabled"].as<bool>();

            if (groundTruth["path"])
                config.evaluation.groundTruth.path =
                    groundTruth["path"].as<std::string>();
        }
    }


    // ============================================================
    // LOGGING
    // ============================================================

    if (auto node = root["logging"])
    {
        if (node["level"])
            config.logging.level =
                node["level"].as<std::string>();


        if (auto modules = node["modules"])
        {
            if (modules["sensor_manager"])
                config.logging.modules.sensorManager =
                    modules["sensor_manager"].as<bool>();

            if (modules["buffer_manager"])
                config.logging.modules.bufferManager =
                    modules["buffer_manager"].as<bool>();

            if (modules["frontend"])
                config.logging.modules.frontend =
                    modules["frontend"].as<bool>();

            if (modules["estimator"])
                config.logging.modules.estimator =
                    modules["estimator"].as<bool>();

            if (modules["backend"])
                config.logging.modules.backend =
                    modules["backend"].as<bool>();

            if (modules["loop_closure"])
                config.logging.modules.loopClosure =
                    modules["loop_closure"].as<bool>();
        }


        if (node["save_logs"])
            config.logging.saveLogs =
                node["save_logs"].as<bool>();

        if (node["path"])
            config.logging.path =
                node["path"].as<std::string>();
    }


    return config;
}

static BackendConfig::Optimization parseBackendOptimization(
    const YAML::Node& node);