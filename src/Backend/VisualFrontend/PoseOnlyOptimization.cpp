#include "Backend/VisualFrontend/PoseOnlyOptimization.h"
#include "Backend/OptimizationProblem/CeresOptimizationProblem.h"

#include <ceres/ceres.h>
#include <Eigen/Geometry>

struct PoseOnlyReprojectionError {
    Eigen::Vector3d point;
    Eigen::Vector2d measurement;
    double fx, fy, cx, cy;

    template<typename T>
    bool operator()(const T* q, const T* t, T* residual) const {
        T p[3] = {T(point.x()) - t[0], T(point.y()) - t[1], T(point.z()) - t[2]};
        const T qInv[4] = {q[0], -q[1], -q[2], -q[3]};
        T qv[3] = {qInv[1], qInv[2], qInv[3]};
        T cross1[3] = {qv[1]*p[2]-qv[2]*p[1], qv[2]*p[0]-qv[0]*p[2], qv[0]*p[1]-qv[1]*p[0]};
        T cross2[3] = {qv[1]*cross1[2]-qv[2]*cross1[1], qv[2]*cross1[0]-qv[0]*cross1[2], qv[0]*cross1[1]-qv[1]*cross1[0]};
        T pc[3] = {p[0]+T(2)*(qInv[0]*cross1[0]+cross2[0]), p[1]+T(2)*(qInv[0]*cross1[1]+cross2[1]), p[2]+T(2)*(qInv[0]*cross1[2]+cross2[2])};
        const T z = pc[2] > T(1e-6) ? pc[2] : T(1e-6);
        residual[0] = T(fx)*pc[0]/z + T(cx) - T(measurement.x());
        residual[1] = T(fy)*pc[1]/z + T(cy) - T(measurement.y());
        return true;
    }
};

bool PoseOnlyOptimization::optimize(CameraPose& pose, const std::vector<Eigen::Vector3d>& worldPoints, const std::vector<Eigen::Vector2d>& measurements, const CameraCalibration& calibration, OptimizationSolver& solver) {
    if (!pose.valid || calibration.fx <= 0 || calibration.fy <= 0 || worldPoints.size() != measurements.size() || worldPoints.size() < 4) return false;

    Eigen::Quaterniond q(pose.worldFromCameraRotation);
    double quaternion[4] = {q.w(), q.x(), q.y(), q.z()};
    double position[3] = {pose.positionWorld.x(), pose.positionWorld.y(), pose.positionWorld.z()};

    CeresOptimizationProblem optimizationProblem;
    auto& problem = optimizationProblem.getProblem();

    problem.AddParameterBlock(quaternion, 4, new ceres::QuaternionManifold());
    problem.AddParameterBlock(position, 3);

    for (std::size_t i = 0; i < worldPoints.size(); ++i) {
        auto* costFunction = new ceres::AutoDiffCostFunction<PoseOnlyReprojectionError, 2, 4, 3>(
            new PoseOnlyReprojectionError{worldPoints[i], measurements[i], calibration.fx, calibration.fy, calibration.cx, calibration.cy});

        problem.AddResidualBlock(costFunction, new ceres::HuberLoss(1.0), quaternion, position);
    }

    if (!solver.solve(optimizationProblem)) return false;

    Eigen::Quaterniond optimizedQuaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    optimizedQuaternion.normalize();

    pose.worldFromCameraRotation = optimizedQuaternion.toRotationMatrix();
    pose.positionWorld = Eigen::Vector3d(position[0], position[1], position[2]);
    return true;
}