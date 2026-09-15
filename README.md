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
                                      SLAM
                                       │
        ┌───────────────┬──────────────┼──────────────┬───────────────┬───────────────┐
        │               │              │              │               │               │
        ▼               ▼              ▼              ▼               ▼               ▼
 Sensor Manager      Frontend      Estimator       Backend       Loop Closure    Visualizer
        │               │              │              │               │               │
        │               ├─ Camera      │              │               ├─ Place        └─ Visualization
        │               ├─ IMU         │              │               │  Recognition
        │               └─ LiDAR       │              │               ├─ Loop Detection
        │                              │              │               └─ Constraints
        │                              │              │
        │                              │              ├── Local Backend
        │                              │              │    ├─ Sliding Window
        │                              │              │    ├─ Local Pose Optimization
        │                              │              │    ├─ Local Bundle Adjustment
        │                              │              │    └─ Marginalization
        │                              │              │
        │                              │              └── Global Backend
        │                              │                   ├─ Pose Graph Optimization
        │                              │                   ├─ Global Bundle Adjustment
        │                              │                   └─ Map / Trajectory Optimization
        │                              │
        │                     ┌────────┴────────┐
        │                     │                 │
        │                     ▼                 ▼
        │                  Filter           Optimizer
        │                     │                 │
        │             ┌───────┼───────┐         ├─ State
        │             │       │       │         ├─ Factors
        │             ▼       ▼       ▼         ├─ Residuals
        │        Predictor  State  Corrector    ├─ Jacobians
        │             │               │         └─ Solver
        │             ▼               ▼              ├─ Ceres
        │       Motion Model   Measurement Model     ├─ GTSAM
        │                                              └─ g2o
        │
        │
        └──────────────────────────────────────────────────────────────────────────────┐
                                                                                       │
                                                                                       ▼
                                                                                  Map Manager
                                                                                       │
                                                                                       ▼
                                                                                  Map / Landmarks
```

### Sensors: Calibration, Drivers, Dataset Player, Data Source

```text
                         Config YAML
                              │
                              ▼
                       SensorManager
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
 SensorContext(s)       BufferManager       CalibrationManager
        │                     ▲                     │
        ▼                     │                     ▼
 SensorDriver            DatasetPlayer       ICalibrationLoader
        │                     ▲                     │
        ▼                     │                     ▼
EurocDatasetDriver       IDataSource       EurocCalibrationLoader
                              │
                         ┌────┴─────┐
                         ▼           ▼
                  EurocCameraSource  EurocImuSource
                         │           │
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
                    ┌─────────────┴─────────────┐
                    │                           │
                    ▼                           ▼
              FilterEstimator          OptimizationEstimator
                    │                           │
          ┌─────────┼─────────┐       ┌─────────┼──────────────┐
          │         │         │       │         │              │
          ▼         ▼         ▼       ▼         ▼              ▼
     FilterState Predictor Corrector  State  SlidingWindow  Optimizer
          │         │         │                 │              │
          │         │         │                 │              ▼
          │         │         │                 │       Nonlinear Solver
          │         │         │                 │
          │         │         │                 ├── Visual Residuals
          │         │         │                 └── IMU Residuals
          │         │         │
          │         │         └── Measurement Update
          │         │
          │         └── Motion / IMU Propagation
          │
          └── Estimated State

                    FilterEstimator
                          │
                          └── Covariance / Uncertainty

              OptimizationEstimator
                          │
                          └── Marginalization
                               └── Schur Complement
```
### Monocular Visual Frontend Pipeline

```text
                         CameraData
                             │
                             ▼
                  ┌─────────────────────┐
                  │  Frame Conversion   │
                  │  → SLAM Frame       │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Image Preprocessing │
                  │ • Undistortion      │
                  │ • Grayscale         │
                  │ • Image Pyramid     │
                  │ • Blur Check        │
                  │ • Exposure Check    │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Feature Detection   │
                  │ & Description       │
                  │        ORB          │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Feature Matching    │
                  │ • BF Matcher        │
                  │ • KNN Matching      │
                  │ • Lowe Ratio Test   │
                  │ • Cross Check       │
                  └──────────┬──────────┘
                             │
                             ▼
                  ┌─────────────────────┐
                  │ Geometric Estimation│
                  │ • RANSAC            │
                  │ • Essential Matrix  │
                  │ • PnP               │
                  │ • Pose Recovery     │
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
                    ▼        ▼        ▼
                  New    Tentative   Active
                    │        │        │
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
                   ▼                   ▼
                  No                  Yes
                   │                   │
                   ▼                   ▼
              Continue              Keyframe
              Tracking                 │
                                       ▼
                              Local Mapping /
                                Optimization
```

## Agentic AI Overview

**SLAM Sentinel** is the project's agentic AI layer for automated engineering validation of the SLAM codebase.

Current implemented capabilities:

- **Project Scanner** — scans the repository and identifies directories, C++, header, YAML, and Python files.
- **Architecture Contract** — defines the expected architecture components and their interface/implementation requirements.
- **Architecture Validator** — validates the presence and basic structural consistency of key architecture components.
- **Architecture Reporting** — generates a validation report showing component status, failures, and warnings.

The current implementation focuses on deterministic architecture validation. The broader SLAM research, trajectory analysis, experiment management, and LLM-based reasoning capabilities are part of the intended future direction.