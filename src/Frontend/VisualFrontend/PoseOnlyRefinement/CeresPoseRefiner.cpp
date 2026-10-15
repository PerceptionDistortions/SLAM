
#include "Frontend/VisualFrontend/PoseOnlyRefinement/CeresPoseRefiner.h"

#include <ceres/ceres.h>
#include <ceres/manifold.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

using DistortionCoefficients = std::array<double, 5>;

DistortionCoefficients extractDistortion(
    const CameraCalibration& calibration,
    std::size_t maxDistortionCoefficients)
{
    DistortionCoefficients distortion{};

    const std::size_t count = std::min(
        calibration.distortion.size(),
        maxDistortionCoefficients);

    for (std::size_t i = 0; i < count; ++i)
    {
        distortion[i] = calibration.distortion[i];
    }

    return distortion;
}

bool isCalibrationValid(
    const CameraCalibration& calibration,
    std::size_t maxDistortionCoefficients)
{
    if (!std::isfinite(calibration.fx) ||
        !std::isfinite(calibration.fy) ||
        !std::isfinite(calibration.cx) ||
        !std::isfinite(calibration.cy))
    {
        return false;
    }

    if (calibration.fx <= 0.0 ||
        calibration.fy <= 0.0)
    {
        return false;
    }

    if (calibration.distortion.size() >
        maxDistortionCoefficients)
    {
        return false;
    }

    for (double coefficient : calibration.distortion)
    {
        if (!std::isfinite(coefficient))
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
// Reprojection residual
//
// Optimized parameters:
//   quaternion: world-to-camera rotation, [x, y, z, w] storage
//   translation: world-to-camera translation, [tx, ty, tz]
//
// Fixed inputs:
//   world landmark, observed image point, camera calibration
// -----------------------------------------------------------------------------

struct ReprojectionResidual
{
    Eigen::Vector3d pointWorld;
    Eigen::Vector2d observedPixel;

    double fx;
    double fy;
    double cx;
    double cy;

    DistortionCoefficients distortion;
    double minimumDepth;

    ReprojectionResidual(
        const PoseCorrespondence& correspondence,
        const CameraCalibration& calibration,
        const DistortionCoefficients& distortionCoefficients,
        double minimumDepth)
        : pointWorld(correspondence.pointWorld),
          observedPixel(correspondence.imagePoint),
          fx(calibration.fx),
          fy(calibration.fy),
          cx(calibration.cx),
          cy(calibration.cy),
          distortion(distortionCoefficients),
          minimumDepth(minimumDepth)
    {
    }

    template <typename T>
    bool operator()(
        const T* const quaternion,
        const T* const translation,
        T* residuals) const
    {
        const Eigen::Quaternion<T> qCameraWorld(
            quaternion[3],
            quaternion[0],
            quaternion[1],
            quaternion[2]);

        const Eigen::Matrix<T, 3, 1> pointWorldT(
            T(pointWorld.x()),
            T(pointWorld.y()),
            T(pointWorld.z()));

        const Eigen::Matrix<T, 3, 1> pointCamera =
            qCameraWorld * pointWorldT +
            Eigen::Matrix<T, 3, 1>(
                translation[0],
                translation[1],
                translation[2]);

        const T x = pointCamera.x();
        const T y = pointCamera.y();
        const T z = pointCamera.z();

        // Keeps division numerically defined during intermediate solver steps.
        // Final poses are separately checked for positive depth.
        const T safeZ =
            z > T(minimumDepth)
                ? z
                : T(minimumDepth);

        const T xn = x / safeZ;
        const T yn = y / safeZ;

        const T r2 = xn * xn + yn * yn;
        const T r4 = r2 * r2;
        const T r6 = r4 * r2;

        const T k1 = T(distortion[0]);
        const T k2 = T(distortion[1]);
        const T p1 = T(distortion[2]);
        const T p2 = T(distortion[3]);
        const T k3 = T(distortion[4]);

        const T radial =
            T(1.0) +
            k1 * r2 +
            k2 * r4 +
            k3 * r6;

        const T xDistorted =
            xn * radial +
            T(2.0) * p1 * xn * yn +
            p2 * (r2 + T(2.0) * xn * xn);

        const T yDistorted =
            yn * radial +
            p1 * (r2 + T(2.0) * yn * yn) +
            T(2.0) * p2 * xn * yn;

        const T predictedU =
            T(fx) * xDistorted + T(cx);

        const T predictedV =
            T(fy) * yDistorted + T(cy);

        residuals[0] =
            predictedU - T(observedPixel.x());

        residuals[1] =
            predictedV - T(observedPixel.y());

        return true;
    }
};

// -----------------------------------------------------------------------------
// Compute RMS reprojection error in pixels.
// -----------------------------------------------------------------------------

bool computeRmsReprojectionError(
    const Eigen::Quaterniond& qCameraWorld,
    const Eigen::Vector3d& tCameraWorld,
    const std::vector<PoseCorrespondence>& correspondences,
    const CameraCalibration& calibration,
    const DistortionCoefficients& distortion,
    double minimumDepth,
    double& rmsError)
{
    if (correspondences.empty())
    {
        return false;
    }

    double squaredErrorSum = 0.0;

    for (const PoseCorrespondence& correspondence :
         correspondences)
    {
        if (!correspondence.isValid())
        {
            return false;
        }

        const Eigen::Vector3d pointCamera =
            qCameraWorld * correspondence.pointWorld +
            tCameraWorld;

        if (!pointCamera.allFinite() ||
            pointCamera.z() <= minimumDepth)
        {
            return false;
        }

        const double xn =
            pointCamera.x() / pointCamera.z();

        const double yn =
            pointCamera.y() / pointCamera.z();

        const double r2 = xn * xn + yn * yn;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double radial =
            1.0 +
            distortion[0] * r2 +
            distortion[1] * r4 +
            distortion[4] * r6;

        const double xDistorted =
            xn * radial +
            2.0 * distortion[2] * xn * yn +
            distortion[3] * (r2 + 2.0 * xn * xn);

        const double yDistorted =
            yn * radial +
            distortion[2] * (r2 + 2.0 * yn * yn) +
            2.0 * distortion[3] * xn * yn;

        const double predictedU =
            calibration.fx * xDistorted + calibration.cx;

        const double predictedV =
            calibration.fy * yDistorted + calibration.cy;

        const double du =
            predictedU - correspondence.imagePoint.x();

        const double dv =
            predictedV - correspondence.imagePoint.y();

        if (!std::isfinite(du) || !std::isfinite(dv))
        {
            return false;
        }

        squaredErrorSum += du * du + dv * dv;
    }

    rmsError = std::sqrt(
        squaredErrorSum /
        static_cast<double>(correspondences.size()));

    return std::isfinite(rmsError);
}

PoseRefinementResult createFailureResult(
    int numCorrespondences,
    const std::string& message)
{
    PoseRefinementResult result;
    result.success = false;
    result.numCorrespondences = numCorrespondences;
    result.message = message;

    return result;
}

// -----------------------------------------------------------------------------
// CeresPoseRefiner
// -----------------------------------------------------------------------------

PoseRefinementResult CeresPoseRefiner::refinePose(
    Eigen::Quaterniond& qCameraWorld,
    Eigen::Vector3d& tCameraWorld,
    const std::vector<PoseCorrespondence>& correspondences,
    const CameraCalibration& calibration) const
{
    // TODO(CONFIG): replace these hardcoded values with the corresponding
    // system.yaml entries and pass a PoseRefinementConfig to this class.
    const int minimumCorrespondences = 20;
    const int maxIterations = 15;
    const double maxSolverTimeSeconds = 0.05;
    const bool useRobustLoss = true;
    const double huberLossScale = 3.0;
    const double functionTolerance = 1e-8;
    const double gradientTolerance = 1e-10;
    const double parameterTolerance = 1e-8;
    const std::string linearSolver = "dense_qr";

    // TODO(CONFIG): make the supported distortion coefficient count and
    // minimum positive depth configurable only if the camera model requires it.
    const std::size_t maxDistortionCoefficients = 5;
    const double minimumDepth = 1e-6;
    const double minimumQuaternionNorm = 1e-12;
    const double improvementTolerance = 1e-9;

    const int numCorrespondences =
        static_cast<int>(correspondences.size());

    if (!isCalibrationValid(
            calibration,
            maxDistortionCoefficients))
    {
        return createFailureResult(
            numCorrespondences,
            "Invalid or unsupported camera calibration.");
    }

    if (numCorrespondences < minimumCorrespondences)
    {
        return createFailureResult(
            numCorrespondences,
            "Insufficient 2D-3D correspondences.");
    }

    if (!qCameraWorld.coeffs().allFinite() ||
        !tCameraWorld.allFinite() ||
        !std::isfinite(qCameraWorld.norm()) ||
        qCameraWorld.norm() < minimumQuaternionNorm)
    {
        return createFailureResult(
            numCorrespondences,
            "Initial camera pose is invalid.");
    }

    for (const PoseCorrespondence& correspondence :
         correspondences)
    {
        if (!correspondence.isValid())
        {
            return createFailureResult(
                numCorrespondences,
                "Invalid correspondence.");
        }
    }

    // Optimize copies so a failed solve cannot overwrite the caller's pose.
    Eigen::Quaterniond refinedQuaternion =
        qCameraWorld.normalized();

    Eigen::Vector3d refinedTranslation =
        tCameraWorld;

    const DistortionCoefficients distortion =
        extractDistortion(
            calibration,
            maxDistortionCoefficients);

    double initialError = 0.0;

    if (!computeRmsReprojectionError(
            refinedQuaternion,
            refinedTranslation,
            correspondences,
            calibration,
            distortion,
            minimumDepth,
            initialError))
    {
        return createFailureResult(
            numCorrespondences,
            "Initial pose has invalid projections or points behind the camera.");
    }

    ceres::Problem problem;

    double* quaternionParameters =
        refinedQuaternion.coeffs().data();

    double* translationParameters =
        refinedTranslation.data();

    problem.AddParameterBlock(
        quaternionParameters,
        4,
        new ceres::EigenQuaternionManifold());

    problem.AddParameterBlock(
        translationParameters,
        3);

    for (const PoseCorrespondence& correspondence :
         correspondences)
    {
        auto* costFunction =
            new ceres::AutoDiffCostFunction<
                ReprojectionResidual, 2, 4, 3>(
                new ReprojectionResidual(
                    correspondence,
                    calibration,
                    distortion,
                    minimumDepth));

        ceres::LossFunction* lossFunction = nullptr;

        if (useRobustLoss)
        {
            // TODO(CONFIG): use ceres_por_use_robust_loss and
            // ceres_por_huber_loss_scale from system.yaml.
            lossFunction = new ceres::HuberLoss(huberLossScale);
        }

        problem.AddResidualBlock(
            costFunction,
            lossFunction,
            quaternionParameters,
            translationParameters);
    }

    ceres::Solver::Options options;

    // TODO(CONFIG): map all these settings from system.yaml.
    options.max_num_iterations = maxIterations;
    options.max_solver_time_in_seconds = maxSolverTimeSeconds;
    options.function_tolerance = functionTolerance;
    options.gradient_tolerance = gradientTolerance;
    options.parameter_tolerance = parameterTolerance;
    options.minimizer_progress_to_stdout = false;

    // TODO(CONFIG): parse the configured string and map it to the
    // corresponding ceres::LinearSolverType enum.
    if (linearSolver == "dense_qr")
    {
        options.linear_solver_type = ceres::DENSE_QR;
    }
    else
    {
        return createFailureResult(
            numCorrespondences,
            "Unsupported Ceres linear solver: " + linearSolver);
    }

    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
        return createFailureResult(
            numCorrespondences,
            "Ceres produced no usable solution: " +
                summary.BriefReport());
    }

    refinedQuaternion.normalize();

    double finalError = 0.0;

    if (!computeRmsReprojectionError(
            refinedQuaternion,
            refinedTranslation,
            correspondences,
            calibration,
            distortion,
            minimumDepth,
            finalError))
    {
        return createFailureResult(
            numCorrespondences,
            "Refined pose has invalid projections or points behind the camera.");
    }

    if (finalError > initialError + improvementTolerance)
    {
        return createFailureResult(
            numCorrespondences,
            "Refinement increased RMS reprojection error.");
    }

    // Commit only after the result passes validation.
    qCameraWorld = refinedQuaternion;
    tCameraWorld = refinedTranslation;

    PoseRefinementResult result;
    result.success = true;
    result.numCorrespondences = numCorrespondences;
    result.initialReprojectionError = initialError;
    result.finalReprojectionError = finalError;
    result.message = summary.BriefReport();

    return result;
}
