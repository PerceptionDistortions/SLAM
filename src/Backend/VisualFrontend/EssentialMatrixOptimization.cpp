
#include "Backend/VisualFrontend/EssentialMatrixOptimization.h"
#include "Backend/OptimizationProblem/CeresOptimizationProblem.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

struct TwoViewReprojectionError {
    double u1, v1, u2, v2;
    double fx, fy, cx, cy;

    template<typename T>
    bool operator()(const T* q, const T* t, const T* point, T* residual) const {
        T rotated[3];
        ceres::QuaternionRotatePoint(q, point, rotated);

        T z1 = point[2] > T(1e-6) ? point[2] : T(1e-6);
        T z2 = rotated[2] + t[2] > T(1e-6) ? rotated[2] + t[2] : T(1e-6);

        residual[0] = T(fx) * point[0] / z1 + T(cx) - T(u1);
        residual[1] = T(fy) * point[1] / z1 + T(cy) - T(v1);
        residual[2] = T(fx) * (rotated[0] + t[0]) / z2 + T(cx) - T(u2);
        residual[3] = T(fy) * (rotated[1] + t[1]) / z2 + T(cy) - T(v2);

        return true;
    }
};

bool EssentialMatrixOptimization::optimize(
    Eigen::Matrix3d& relativeRotation,
    Eigen::Vector3d& translationDirection,
    std::vector<Eigen::Vector3d>& landmarks3D,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    const CameraCalibration& calibration,
    OptimizationSolver& solver) {

    if (calibration.fx <= 0 || calibration.fy <= 0 ||
        points1.size() != points2.size() ||
        points1.size() != landmarks3D.size() ||
        points1.size() < 5 ||
        translationDirection.norm() < 1e-8) return false;

    Eigen::Quaterniond q(relativeRotation);
    q.normalize();
    translationDirection.normalize();

    double quaternion[4] = {q.w(), q.x(), q.y(), q.z()};
    double translation[3] = {
        translationDirection.x(),
        translationDirection.y(),
        translationDirection.z()
    };

    std::vector<std::array<double, 3>> landmarkParameters;
    landmarkParameters.reserve(landmarks3D.size());

    for (const auto& landmark : landmarks3D) {
        landmarkParameters.push_back({
            landmark.x(), landmark.y(), landmark.z()
        });
    }

    CeresOptimizationProblem optimizationProblem;
    auto& problem = optimizationProblem.getProblem();

    problem.AddParameterBlock(quaternion, 4, new ceres::QuaternionManifold());
    problem.AddParameterBlock(translation, 3, new ceres::SphereManifold<3>());

    for (auto& landmark : landmarkParameters) {
        problem.AddParameterBlock(landmark.data(), 3);
    }

    for (std::size_t i = 0; i < points1.size(); ++i) {
        auto* costFunction =
            new ceres::AutoDiffCostFunction<TwoViewReprojectionError, 4, 4, 3, 3>(
                new TwoViewReprojectionError{
                    points1[i].x(), points1[i].y(),
                    points2[i].x(), points2[i].y(),
                    calibration.fx, calibration.fy,
                    calibration.cx, calibration.cy
                });

        problem.AddResidualBlock(
            costFunction,
            new ceres::HuberLoss(1.0),
            quaternion,
            translation,
            landmarkParameters[i].data());
    }

    if (!solver.solve(optimizationProblem)) return false;

    Eigen::Quaterniond optimizedQuaternion(
        quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    optimizedQuaternion.normalize();

    relativeRotation = optimizedQuaternion.toRotationMatrix();
    translationDirection =
        Eigen::Vector3d(translation[0], translation[1], translation[2]).normalized();

    for (std::size_t i = 0; i < landmarks3D.size(); ++i) {
        landmarks3D[i] = Eigen::Vector3d(
            landmarkParameters[i][0],
            landmarkParameters[i][1],
            landmarkParameters[i][2]);
    }

    return true;
}
