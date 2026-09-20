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
