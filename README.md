# SLAM (Simultaneous Localization and Mapping)

A modular C++17 multi-sensor SLAM system with automated evaluation and failure analysis.

## System Specification

- **Language:** C++17

- **Computer Vision:** OpenCV

- **Linear Algebra:** Eigen

- **Configuration:** YAML

- **Visualizer:** Pangolin

- **Dataset:** EuRoC MAV

## Status

- ⚪ Not Started

- 🟡 In Progress

- 🟢 Completed

## Target Systems

- 🟡 Monocular SLAM

- ⚪ Monocular VI-SLAM

- ⚪ Stereo VI-SLAM

| System | Sensors | Primary Estimator | Prediction | Backend |

|---|---|---|---|---|

| **Monocular SLAM** | 1× Camera | EKF | Constant Velocity | Local + Global Optimization |

| **Monocular VI-SLAM** | 1× Camera + IMU | Sliding-Window Optimization | IMU Propagation / Preintegration | Global Optimization |

| **Stereo VI-SLAM** | Stereo Camera + IMU | Sliding-Window Optimization | IMU Propagation / Preintegration | Global Optimization |

## Architecture Layers

- 🟢 **Configuration** — YAML-based system and sensor configuration

- 🟢 **Sensor Management** — Sensor registration and lifecycle coordination

- 🟢 **Sensor Context** — Per-sensor driver and calibration ownership

- 🟢 **Calibration** — Calibration management and loading

- 🟢 **Sensor Drivers** — Sensor lifecycle abstraction

- 🟢 **Data Sources** — EuRoC camera and IMU data acquisition

- 🟢 **Dataset Player** — Dataset playback and measurement delivery

- 🟢 **Buffer Management** — Bounded, timestamp-aware multi-sensor buffering

- 🟡 **Visual Frontend** — Monocular visual processing

- ⚪ **IMU Frontend** — IMU processing and preintegration

- ⚪ **LiDAR Frontend**

- ⚪ **SLAM Backend** — Mapping and optimization

- ⚪ **Trajectory Evaluation** — ATE, RPE and trajectory analysis

- 🟡 **SLAM Sentinel** — Automated architecture and engineering validation

## Visual Frontend

- ⚪ Feature detection using ORB

- ⚪ Feature matching

- ⚪ Descriptor filtering

- ⚪ Lowe Ratio Test

- ⚪ RANSAC

- ⚪ Essential Matrix

- ⚪ Fundamental Matrix

- ⚪ PnP

- ⚪ Pose recovery

- ⚪ Motion gating

- ⚪ Triangulation

- ⚪ Parallax check

- ⚪ Reprojection error check

- ⚪ Pose refinement

- ⚪ Track management

- ⚪ Landmark management

- ⚪ Keyframe selection

- ⚪ Local mapping

- ⚪ Local optimization

## High-Level Architecture

### SLAM Core Architecture

```text
                                  ┌──────────────────────────┐
                                  │      Configuration       │
                                  │   YAML / sensor setup    │
                                  │   SLAM type / params     │
                                  └────────────┬─────────────┘
                                               │
                                               ▼
                                  ┌──────────────────────────┐
                                  │       SLAM Factory       │
                                  │  creates SLAM system from│
                                  │       configuration      │
                                  └────────────┬─────────────┘
                                               │
                         ┌─────────────────────┼─────────────────────┐
                         ▼                     ▼                     ▼
                  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐
                  │  Monocular   │      │  Monocular   │      │   Stereo     │
                  │    SLAM      │      │   VI-SLAM    │      │   VI-SLAM    │
                  │   Camera     │      │ Camera + IMU │      │ Stereo + IMU │
                  └──────┬───────┘      └──────┬───────┘      └──────┬───────┘
                         └─────────────────────┼──────────────────────┘
                                               ▼
                                  ┌──────────────────────────┐
                                  │       SLAM System        │
                                  │      Orchestrator        │
                                  │                          │
                                  │ init / process / run     │
                                  │ module lifecycle & flow  │
                                  └────────────┬─────────────┘
                                               │
                                               ▼
                                  ┌──────────────────────────┐
                                  │      Buffer Manager      │
                                  │                          │
                                  │ chronological sensor     │
                                  │ packets / timestamps     │
                                  │ synchronization          │
                                  └────────────┬─────────────┘
                                               │
                                               ▼
                         ┌─────────────────────┴─────────────────────┐
                         │                  Frontend                 │
                         │            sensor processing              │
                         └─────────────────────┬─────────────────────┘
                                               │
                ┌──────────────────────────────┼──────────────────────────────┐
                ▼                              ▼                              ▼
       ┌─────────────────┐            ┌─────────────────┐            ┌─────────────────┐
       │ Visual Frontend │            │  IMU Frontend   │            │ LiDAR Frontend  │
       │                 │            │                 │            │   (optional)    │
       │ feature detect  │            │ bias / noise    │            │ preprocessing   │
       │ tracking        │            │ preprocessing   │            │ scan matching   │
       │ matching        │            │ integration /   │            │ geometric       │
       │ motion estimate │            │ preintegration  │            │ features        │
       │ landmarks       │            │                 │            │                 │
       └────────┬────────┘            └────────┬────────┘            └────────┬────────┘
                │                              │                              │
                └──────────────────────────────┼──────────────────────────────┘
                                               ▼
                                  ┌──────────────────────────┐
                                  │       Measurement        │
                                  │                          │
                                  │ IMU / Visual / Stereo    │
                                  │ LiDAR / other sensors    │
                                  └────────────┬─────────────┘
                                               │
                                               ▼
                                  ┌──────────────────────────┐
                                  │        Estimator         │
                                  │     State Estimation     │
                                  │                          │
                                  │ process(measurement)     │
                                  │ → state + covariance     │
                                  └────────────┬─────────────┘
                                               │
                         ┌─────────────────────┴─────────────────────┐
                         ▼                                           ▼
              ┌──────────────────────┐                   ┌────────────────────────┐
              │   Filter Estimator   │                   │ Optimization Estimator │
              │       (ESEKF)        │                   │                        │
              └──────────┬───────────┘                   │ sliding-window / VIO   │
                         │                               │ state & factor update  │
              ┌──────────┴───────────┐                   └───────────┬─────────── ┘
              ▼                      ▼                               │
      ┌────────────────┐    ┌────────────────┐                       │
      │   Prediction   │    │   Correction   │                       │
      │   Strategies   │    │    Strategy    │                       │
      ├────────────────┤    ├────────────────┤                       │
      │ Constant       │    │ Visual         │                       │
      │ Velocity       │    │ Frontend       │                       │
      │ Model          │    │ Measurement    │                       │
      │                │    │                │                       │
      │ IMU            │    │ Mono / Stereo  │                       │
      │ Propagation    │    │                │                       │
      └───────┬────────┘    └───────┬────────┘                       │
              │                     │                                │
              └──────────┬──────────┘                                │
                         ▼                                           │
                  State + Covariance                                 │
                         │                                           │
                         └─────────────────────┬─────────────────────┘
                                               │
                                               ▼
                                  ┌──────────────────────────┐
                                  │         Backend          │
                                  │     Map Optimization     │
                                  └────────────┬─────────────┘
                                               │
                     ┌─────────────────────────┼─────────────────────────┐
                     ▼                         ▼                         ▼
          ┌────────────────────┐    ┌────────────────────┐    ┌────────────────────┐
          │ Local Optimization │    │ Global Optimization│    │ Pose Graph         │
          │                    │    │                    │    │ Optimization       │
          │ recent states      │    │ full / large-scale │    │                    │
          │ keyframes          │    │ refinement         │    │ poses + constraints│
          │ local map          │    │                    │    │                    │
          │                    │    │                    │    │                    │
          │ GN / LM            │    │ GN / LM / Dogleg   │    │ GN / LM / iSAM2    │
          │                    │    │ / iSAM2            │    │ / g2o / Ceres      │
          └──────────┬─────────┘    └──────────┬─────────┘    └──────────┬─────────┘
                     │                         │                         │
                     └─────────────────────────┼─────────────────────────┘
                                               │
                                               ▼
                                  ┌──────────────────────────┐
                                  │      Loop Closure        │
                                  │                          │
                                  │ place recognition        │
                                  │ geometric verification   │
                                  │ loop constraint creation │
                                  └────────────┬─────────────┘
                                               │
                                               │ loop constraints
                                               ▼
                                  ┌──────────────────────────┐
                                  │    Pose Graph / Global   │
                                  │       Optimization       │
                                  └────────────┬─────────────┘
                                               │
                                               ▼
                                  ┌──────────────────────────┐
                                  │      Map & Outputs       │
                                  │                          │
                                  │ optimized trajectory     │
                                  │ landmarks / point cloud  │
                                  │ keyframes                │
                                  │ covariance / metrics     │
                                  └──────────────────────────┘
```

### Sensors: Calibration, Drivers, Dataset Player, Data Source

```text
                         Config YAML
                              │
                              ▼
                       SensorManager
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
 SensorContext(s)       BufferManager       CalibrationManager
        │                     ▲                     │
        ▼                     │                     ▼
 SensorDriver            DatasetPlayer       ICalibrationLoader
        │                     ▲                     │
        ▼                     │                     ▼
EurocDatasetDriver       IDataSource       EurocCalibrationLoader
                              │
                         ┌────┴─────┐
                         ▼           ▼
                  EurocCameraSource  EurocImuSource
                         │           │
                         └────┬──────┘
                              ▼
                         BufferManager
                              │
                              ▼
                          SLAM Core
```

### Estimator Architecture

```text
                               Estimator
                                  │
                 ┌────────────────┴────────────────┐
                 │                                 │
                 ▼                                 ▼
        ┌──────────────────┐              ┌────────────────────────┐
        │ Filter Estimator │              │ Optimization Estimator │
        │      ESEKF       │              │                        │
        └────────┬─────────┘              │ Sliding Window / VIO   │
                 │                        │ State + Factors        │
        ┌────────┴────────┐               └───────────┬────────────┘
        ▼                 ▼                           │
 ┌─────────────┐   ┌─────────────┐                    │
 │ Prediction  │   │ Correction  │                    │
 │ Strategies  │   │  Strategy   │                    │
 ├─────────────┤   ├─────────────┤                    │
 │ Constant    │   │ Visual      │                    │
 │ Velocity    │   │ Measurement │                    │
 │             │   │             │                    │
 │ IMU         │   │ Mono /      │                    │
 │ Propagation │   │ Stereo      │                    │
 └──────┬──────┘   └──────┬──────┘                    │
        │                 │                           │
        └─────────┬───────┘                           │
                  ▼                                   ▼
           State + Covariance                 State + Factors
                  │                                    │
                  ▼                                    ▼
             Filter Result                     Optimization Result
```

### Monocular Visual Frontend Pipeline

```text
                         CameraData
                             │
                             ▼
                  ┌─────────────────────┐
                  │  Frame Conversion   │
                  │  → SLAM Frame       │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Image Preprocessing │
                  │ • Undistortion      │
                  │ • Grayscale         │
                  │ • Image Pyramid     │
                  │ • Blur Check        │
                  │ • Exposure Check    │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Feature Detection   │
                  │ & Description       │
                  │        ORB          │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Feature Matching    │
                  │ • BF Matcher        │
                  │ • KNN Matching      │
                  │ • Lowe Ratio Test   │
                  │ • Cross Check       │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Geometric Estimation│
                  │ • RANSAC            │
                  │ • Essential Matrix  │
                  │ • PnP               │
                  │ • Pose Recovery     │
                  └──────────┬──────────┘
                             │
                             ▼
                     Motion Validation
                             │
                             ▼
                      Pose Refinement
                             │
                             ▼
                      Track Manager
                    ┌────────┼────────┐
                    ▼        ▼        ▼
                  New    Tentative   Active
                    │        │        │
                    └────────┴────────┘
                             │
                             ▼
                       Triangulation
                             │
                             ▼
                   Landmark Validation
                   • Reprojection Error
                   • Positive Depth
                   • Parallax
                   • Geometric Consistency
                             │
                             ▼
                     Landmark Manager
                             │
                             ▼
                     Keyframe Decision
                             │
                   ┌─────────┴─────────┐
                   ▼                   ▼
                  No                  Yes
                   │                   │
                   ▼                   ▼
              Continue              Keyframe
              Tracking                 │
                                       ▼
                              Local Mapping /
                                Optimization
```

### Backend Architecture

```text
                           Backend
                              │
          ┌───────────────────┼────────────────────┐
          │                   │                    │
          ▼                   ▼                    ▼
 ┌────────────────┐   ┌────────────────┐   ┌────────────────────┐
 │ Local          │   │ Global         │   │ Pose Graph         │
 │ Optimization   │   │ Optimization   │   │ Optimization       │
 └───────┬────────┘   └───────┬────────┘   └──────────┬─────────┘
         │                    │                       │
         ▼                    ▼                       ▼
 Recent states          Full / large-scale       Keyframe poses
 Local keyframes        map refinement           Loop constraints
 Local map              trajectory refinement    Global consistency
         │                    │                       │
         ▼                    ▼                       ▼
   ┌───────────┐        ┌───────────┐          ┌───────────┐
   │ GN / LM   │        │ GN / LM   │          │ GN / LM   │
   │           │        │ Dogleg    │          │ Dogleg    │
   └───────────┘        │ iSAM2     │          │ iSAM2     │
                        └───────────┘          │ g2o/Ceres │
                                               └───────────┘
```

## Agentic AI Overview

**SLAM Sentinel** is the project's agentic AI layer for automated engineering validation of the SLAM codebase.

Current implemented capabilities:

- **Project Scanner** — scans the repository and identifies directories, C++, header, YAML, and Python files.

- **Architecture Contract** — defines the expected architecture components and their interface/implementation requirements.

- **Architecture Validator** — validates the presence and basic structural consistency of key architecture components.

- **Architecture Reporting** — generates a validation report showing component status, failures, and warnings.

The current implementation focuses on deterministic architecture validation. The broader SLAM research, trajectory analysis, experiment management, and LLM-based reasoning capabilities are part of the intended future direction.


Visual Track Management
1. Tentative Track
    Age stored and increased with each frame.
    If age limit crossed without promoting or demoting without consecutive misses, pruned.
    If consecutive misses limit crossed, deleted track.
    If consecutive matches as defined, promotes to active track.
2. Active Track
    Age stored and increased with each frame.
    If age limit crossed without promoting or demoting without consecutive misses, pruned.
    If consecutive misses limit crossed, demoted to tentative.
    If consecutive matches as defined and passes parallax and baseline test, triangulation is attempted and if success and validated, it is promoted to a triangulated track.
3. Triangulated track
    No age based pruning.
    consective failures, marked as triagnulated_lost state.
    If passes the multi frame matching and validation and reprojection tests, should be promoted to landmkark.
    Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.
4. Triangulated_lost
    No age stored: only consecutive matches or misses
    If consecutuce macthes: promoted back to triangulated. Needs correspondence match, not only feature match.
    If consecutive misses: deleted (not demoetd to active).
    Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.
5. Landmark
    Age is not stored and so no pruning.
    Consecutive misses: demote to landmark lost.
    Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.
6. Landmark_lost
    No age stored
    Consecutive misses: delete
    Consecitve matches: promote back to landmark.
     Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.

    
    Challenges faced in track management:
    1. Very low number of active tracks even when the number matches were high. Threshold issue in config.
    2. The active tracks in second frame itself dropped by 60 when only 8 were succesfully triangulated. The fail threshold is 8. Inliers count was not being updated on promotion from tentative track. Every frame check deleted active track, triangulated, and promoted from tentative.
    3. Parallax and baseline check was not checking for consecutive active inliers.

    Failure modes
    1. Scale ambiguity in monocular SLAM.
    2. Degenerate camera motion.
    3. Low parallax and poor triangulation.
    4. Feature tracking degradation.
    5. Incorrect pose estimation.
    6. Covariance inconsistency and estimator divergence.

    Performances
    1. Real-time processing constraints.
    2. CPU and memory profiling.
    3. Multithreading and synchronization.
    4. Embedded/ARM optimization.
    5. Processing latency and throughput.

Challenges faced
1. 100 triangulation candidates, triangulated, 0 passed valdiation: The parallax checks in triangulation and initial parallax check thresholds are diff.
2. Parallax : Baseline not to be used without pnp, arbitrary and normalized scale
3. Parallax threshold relaxation brings better results
4. Checked low trigulation candidates with ground truth pose comparison: relevant
5. Parallax: Do not check parallax from previous frame but from the last keyframe of current track
1. Motion gating: Ratio in pnp was being checked wuith all descriptors instead of only landmark


    
