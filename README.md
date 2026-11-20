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


## Visual Track Management

### 1. Tentative Track

- Age stored and increased with each frame.
- If age limit crossed without promoting or demoting without consecutive misses, pruned.
- If consecutive misses limit crossed, deleted track.
- If consecutive matches as defined, promotes to active track.

### 2. Active Track

- Age stored and increased with each frame.
- If age limit crossed without promoting or demoting without consecutive misses, pruned.
- If consecutive misses limit crossed, demoted to tentative.
- If consecutive matches as defined and passes parallax and baseline test, triangulation is attempted and if success and validated, it is promoted to a triangulated track.

### 3. Triangulated track

- No age based pruning.
- consective failures, marked as triagnulated_lost state.
- If passes the multi frame matching and validation and reprojection tests, should be promoted to landmkark.
- Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.

### 4. Triangulated_lost

- No age stored: only consecutive matches or misses
- If consecutuce macthes: promoted back to triangulated. Needs correspondence match, not only feature match.
- If consecutive misses: deleted (not demoetd to active).
- Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.

### 5. Landmark

- Age is not stored and so no pruning.
- Consecutive misses: demote to landmark lost.
- Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.

### 6. Landmark_lost

- No age stored
- Consecutive misses: delete
- Consecitve matches: promote back to landmark.
- Always to be checked aganst camera pose before conseuctive failures or macthes if it is possible to view landmark from a given cam pose.

    
## Challenges Faced

### Track-Management Challenges

- **Low active-track retention:** The frontend often retained few active tracks despite a high descriptor-match count, suggesting that match filtering and track-transition thresholds needed to be tuned together.
- **Unexpected track loss after promotion:** In one run, the active-track count fell by about 60 in the second frame while only eight tracks triangulated successfully; the miss threshold was also eight. The inlier count was not carried over when a tentative track became active, so subsequent updates could remove it prematurely and repeatedly promote replacement tracks.
- **Insufficient temporal evidence for triangulation:** Parallax and baseline checks were not consistently conditioned on consecutive inlier observations from an active track.

### Initialization, Pose, and Scale Challenges

- **Monocular scale ambiguity:** Essential-matrix pose recovery determines translation direction, not translation magnitude. Position and velocity therefore cannot be recovered metrically from essential-matrix/recover-pose updates alone.
- **Scale consistency between initialization and PnP:** A map initialized from essential-matrix motion may use an arbitrary unit-baseline scale. If later poses and landmarks do not preserve that same scale convention, PnP can become unreliable; backend bundle adjustment under a fixed gauge is needed to refine consistency.
- **PnP does not repair an inconsistent map:** PnP estimates the current camera pose relative to existing landmarks. It does not, by itself, correct landmarks previously triangulated under an inconsistent essential-matrix scale; those points need consistent re-triangulation or map optimization.
- **PnP motion-gating ratio:** The PnP inlier ratio was being computed against all descriptor matches rather than only valid 2D–3D landmark correspondences. The denominator should reflect the correspondence set actually passed to PnP.
- **Pose correction ownership:** The visual frontend's frame pose was not updated from the estimator's corrected pose. The system needs a clear source of truth and a defined path for propagating corrections between estimator, frontend, and map.
- **Backend refinement:** Monocular scale ambiguity and accumulated pose/landmark error make backend map refinement an important part of this system; essential-matrix initialization alone is not a substitute for bundle adjustment.

### Triangulation and Parallax Challenges

- **Candidate/validation threshold mismatch:** One run produced roughly 100 triangulation candidates and no points that passed validation. A likely cause was that candidate selection and triangulation validation used different parallax thresholds; these checks should share a consistent geometric policy.
- **Baseline under unknown scale:** A metric baseline threshold should not be applied directly to translation recovered from essential-matrix geometry, whose magnitude is arbitrary. Use a scale-aware baseline only after the map scale is established, or use a scale-independent parallax criterion.
- **Threshold trade-off:** Relaxing the parallax threshold improved triangulation yield in observed runs, but low-parallax points have uncertain depth. Tune the threshold using cheirality, reprojection error, and ray-separation checks rather than yield alone.
- **Reference-pose choice:** For keyframe mapping, evaluate parallax relative to the track's reference keyframe when appropriate, rather than always using the immediately previous frame.
- **Ground-truth comparison:** Low triangulation counts should be investigated alongside ground-truth pose comparisons where available, so frontend thresholds can be distinguished from pose-estimation error.

### Estimator and Timing Challenges

- **Timestamp units:** Sensor timestamps, frontend timestamps, and estimator state timestamps must use the same unit and epoch. Convert explicitly at a single system boundary.
- **Covariance consistency:** Scale ambiguity, poor covariance initialization, or mismatched process and measurement noise can make the estimator overconfident or cause divergence.
- **Visual-only velocity observability:** Essential-matrix motion provides relative rotation and translation direction, not reliable metric position or velocity. Velocity needs a suitable motion model and additional observable information.

## General Failure Modes and Performance Considerations

### General Failure Modes

- Monocular scale ambiguity and gauge freedom.
- Degenerate camera motion, including near-pure rotation.
- Low parallax leading to poorly conditioned triangulation and uncertain depth.
- Feature-tracking degradation from blur, exposure changes, occlusion, or weak texture.
- Incorrect pose estimates caused by outliers or insufficient geometric constraints.
- Inconsistent covariance estimates and estimator divergence.

### Performance and Deployment Considerations

- Meet real-time processing constraints at the target image resolution and frame rate.
- Profile CPU and memory use on representative sequences.
- Add multithreading only with clear synchronization for shared map and track data.
- Evaluate performance on embedded/ARM hardware if it is a deployment target.
- Measure end-to-end processing latency and sustained throughput.
