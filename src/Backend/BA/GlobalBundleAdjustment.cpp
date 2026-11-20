#include "Backend/BA/GlobalBundleAdjustment.h"
#include "Backend/OptimizationProblem/CeresOptimizationProblem.h"

#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <unordered_map>
#include <vector>
#include <memory>

struct PoseBlock {
    std::array<double, 4> quaternion;
    std::array<double, 3> position;
    std::shared_ptr<Keyframe> keyframe;
};

struct LandmarkBlock {
    std::array<double, 3> position;
    std::shared_ptr<Landmark> landmark;
};

struct ReprojectionError {
    double u, v, fx, fy, cx, cy;

    template<typename T>
    bool operator()(const T* q, const T* t, const T* point, T* residual) const {
        T p[3] = {point[0] - t[0], point[1] - t[1], point[2] - t[2]};
        const T qInv[4] = {q[0], -q[1], -q[2], -q[3]};
        T qv[3] = {qInv[1], qInv[2], qInv[3]};

        T cross1[3] = {
            qv[1] * p[2] - qv[2] * p[1],
            qv[2] * p[0] - qv[0] * p[2],
            qv[0] * p[1] - qv[1] * p[0]
        };

        T cross2[3] = {
            qv[1] * cross1[2] - qv[2] * cross1[1],
            qv[2] * cross1[0] - qv[0] * cross1[2],
            qv[0] * cross1[1] - qv[1] * cross1[0]
        };

        T pc[3] = {
            p[0] + T(2) * (qInv[0] * cross1[0] + cross2[0]),
            p[1] + T(2) * (qInv[0] * cross1[1] + cross2[1]),
            p[2] + T(2) * (qInv[0] * cross1[2] + cross2[2])
        };

        const T z = pc[2] > T(1e-6) ? pc[2] : T(1e-6);
        residual[0] = T(fx) * pc[0] / z + T(cx) - T(u);
        residual[1] = T(fy) * pc[1] / z + T(cy) - T(v);
        return true;
    }
};

bool GlobalBundleAdjustment::optimize(MapManager& map, const CameraCalibration& calibration, OptimizationSolver& solver) {
    if (calibration.fx <= 0 || calibration.fy <= 0) return false;

    auto keyframes = map.getAllKeyframes();
    if (keyframes.size() < 2 || map.getLandmarkCount() == 0) return false;

    std::sort(keyframes.begin(), keyframes.end(), [](const auto& a, const auto& b) {
        return a->getTimestamp() < b->getTimestamp();
    });

    std::vector<PoseBlock> poses;
    poses.reserve(keyframes.size());

    for (const auto& keyframe : keyframes) {
        if (!keyframe || !keyframe->getPose().valid) continue;

        const auto& pose = keyframe->getPose();
        Eigen::Quaterniond q(pose.worldFromCameraRotation);

        poses.push_back({
            {q.w(), q.x(), q.y(), q.z()},
            {pose.positionWorld.x(), pose.positionWorld.y(), pose.positionWorld.z()},
            keyframe
        });
    }

    if (poses.size() < 2) return false;

    Eigen::Vector3d firstPosition(poses[0].position[0], poses[0].position[1], poses[0].position[2]);
    Eigen::Vector3d secondPosition(poses[1].position[0], poses[1].position[1], poses[1].position[2]);

    if ((secondPosition - firstPosition).norm() < 1e-8) return false;

    std::vector<LandmarkBlock> landmarks;
    landmarks.reserve(map.getLandmarkCount());

    std::unordered_map<LandmarkId, std::size_t> landmarkLookup;

    for (const auto& pose : poses) {
        for (const auto& observation : pose.keyframe->getObservations()) {
            const LandmarkId id = observation.getLandmarkId();
            if (landmarkLookup.count(id)) continue;

            auto landmark = map.getLandmark(id);
            if (!landmark || !landmark->isValid()) continue;

            const auto& p = landmark->getPosition();
            landmarkLookup[id] = landmarks.size();
            landmarks.push_back({{p.x(), p.y(), p.z()}, landmark});
        }
    }

    if (landmarks.empty()) return false;

    CeresOptimizationProblem optimizationProblem;
    auto& problem = optimizationProblem.getProblem();

    for (auto& pose : poses) {
        problem.AddParameterBlock(pose.quaternion.data(), 4, new ceres::QuaternionManifold());
        problem.AddParameterBlock(pose.position.data(), 3);
    }

    for (auto& landmark : landmarks) {
        problem.AddParameterBlock(landmark.position.data(), 3);
    }

    problem.SetParameterBlockConstant(poses[0].quaternion.data());
    problem.SetParameterBlockConstant(poses[0].position.data());
    problem.SetParameterBlockConstant(poses[1].position.data());

    std::size_t residualCount = 0;

    for (auto& pose : poses) {
        for (const auto& observation : pose.keyframe->getObservations()) {
            auto it = landmarkLookup.find(observation.getLandmarkId());
            if (it == landmarkLookup.end()) continue;

            const auto& measurement = observation.getMeasurement();

            auto* costFunction = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(
                new ReprojectionError{
                    measurement.x(), measurement.y(),
                    calibration.fx, calibration.fy,
                    calibration.cx, calibration.cy
                });

            problem.AddResidualBlock(
                costFunction,
                new ceres::HuberLoss(1.0),
                pose.quaternion.data(),
                pose.position.data(),
                landmarks[it->second].position.data()
            );

            ++residualCount;
        }
    }

    if (residualCount == 0) return false;
    if (!solver.solve(optimizationProblem)) return false;

    for (auto& pose : poses) {
        Eigen::Quaterniond q(
            pose.quaternion[0], pose.quaternion[1],
            pose.quaternion[2], pose.quaternion[3]
        );
        q.normalize();

        CameraPose updatedPose;
        updatedPose.worldFromCameraRotation = q.toRotationMatrix();
        updatedPose.positionWorld = Eigen::Vector3d(
            pose.position[0], pose.position[1], pose.position[2]
        );
        updatedPose.valid = true;

        pose.keyframe->setPose(updatedPose);
    }

    for (auto& landmark : landmarks) {
        landmark.landmark->setPosition(Eigen::Vector3d(
            landmark.position[0], landmark.position[1], landmark.position[2]
        ));
    }

    return true;
}