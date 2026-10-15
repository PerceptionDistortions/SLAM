#include "Frontend/VisualFrontend/Triangulation/MidpointTriangulator.h"

#include <algorithm>
#include <cmath>
#include <Eigen/LU>

namespace
{
constexpr double kRotationTolerance = 1e-3;
constexpr double kPoseTolerance = 1e-6;
constexpr double kMinimumIntrinsicDeterminant = 1e-12;

bool isValidPose(const Eigen::Matrix4d& Tcw)
{
    if (!Tcw.allFinite())
    {
        return false;
    }

    if ((Tcw.row(3) -
         Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm()
        > kPoseTolerance)
    {
        return false;
    }

    const Eigen::Matrix3d R = Tcw.block<3, 3>(0, 0);

    if ((R.transpose() * R -
         Eigen::Matrix3d::Identity()).norm() > kRotationTolerance)
    {
        return false;
    }

    return std::abs(R.determinant() - 1.0) <= kRotationTolerance;
}

bool isValidCalibration(const Eigen::Matrix3d& K)
{
    if (!K.allFinite())
    {
        return false;
    }

    if (std::abs(K.determinant()) < kMinimumIntrinsicDeterminant)
    {
        return false;
    }

    return K(0, 0) > 0.0 && K(1, 1) > 0.0;
}

bool isValidPixel(const cv::Point2d& pixel)
{
    return std::isfinite(pixel.x) && std::isfinite(pixel.y);
}

bool projectPoint(
    const Eigen::Vector3d& pointCamera,
    const Eigen::Matrix3d& K,
    cv::Point2d& projectedPixel)
{
    if (!pointCamera.allFinite() || pointCamera.z() <= 0.0)
    {
        return false;
    }

    const Eigen::Vector3d projected = K * pointCamera;

    if (!projected.allFinite() ||
        std::abs(projected.z()) < 1e-12)
    {
        return false;
    }

    projectedPixel.x = projected.x() / projected.z();
    projectedPixel.y = projected.y() / projected.z();

    return std::isfinite(projectedPixel.x) &&
           std::isfinite(projectedPixel.y);
}

double pixelError(
    const cv::Point2d& measured,
    const cv::Point2d& projected)
{
    const double dx = measured.x - projected.x;
    const double dy = measured.y - projected.y;

    return std::sqrt(dx * dx + dy * dy);
}
} // namespace

MidpointTriangulator::MidpointTriangulator(
    MidpointTriangulatorConfig config)
    : config_(config)
{
    // Invalid configurations are rejected when triangulate() is called.
}

TriangulationResult MidpointTriangulator::triangulate(
    const TriangulationObservation& first,
    const TriangulationObservation& second) const
{
    TriangulationResult result;

    const auto fail = [&result](TriangulationFailure reason)
    {
        result.failure = reason;
        return result;
    };

    if (!isValidPixel(first.pixel) ||
        !isValidPixel(second.pixel) ||
        !std::isfinite(config_.minParallaxDegrees) ||
        !std::isfinite(config_.minBaseline) ||
        !std::isfinite(config_.maxReprojectionErrorPixels) ||
        !std::isfinite(config_.maxRaySeparation) ||
        !std::isfinite(config_.minDepth) ||
        !std::isfinite(config_.geometryEpsilon) ||
        config_.minParallaxDegrees < 0.0 ||
        config_.minParallaxDegrees >= 90.0 ||
        config_.minBaseline < 0.0 ||
        config_.maxReprojectionErrorPixels < 0.0 ||
        config_.maxRaySeparation < 0.0 ||
        config_.minDepth <= 0.0 ||
        config_.geometryEpsilon <= 0.0)
    {
        return fail(TriangulationFailure::InvalidInput);
    }

    if (!isValidCalibration(first.K) ||
        !isValidCalibration(second.K))
    {
        return fail(TriangulationFailure::InvalidCalibration);
    }

    if (!isValidPose(first.Tcw) || !isValidPose(second.Tcw))
    {
        return fail(TriangulationFailure::InvalidPose);
    }

    // Tcw maps world coordinates into camera coordinates:
    // p_c = R_cw * p_w + t_cw.
    const Eigen::Matrix3d Rcw1 = first.Tcw.block<3, 3>(0, 0);
    const Eigen::Matrix3d Rcw2 = second.Tcw.block<3, 3>(0, 0);

    const Eigen::Vector3d tcw1 = first.Tcw.block<3, 1>(0, 3);
    const Eigen::Vector3d tcw2 = second.Tcw.block<3, 1>(0, 3);

    // Camera centers expressed in world coordinates.
    const Eigen::Vector3d C1 = -Rcw1.transpose() * tcw1;
    const Eigen::Vector3d C2 = -Rcw2.transpose() * tcw2;

    const Eigen::Vector3d baseline = C2 - C1;
    const double baselineLength = baseline.norm();

    if (!std::isfinite(baselineLength) ||
        baselineLength <= config_.minBaseline)
    {
        return fail(TriangulationFailure::InsufficientBaseline);
    }

    // Convert undistorted pixel coordinates into unit bearing vectors.
    const Eigen::Vector3d pixel1(
        first.pixel.x, first.pixel.y, 1.0);

    const Eigen::Vector3d pixel2(
        second.pixel.x, second.pixel.y, 1.0);

    Eigen::Vector3d bearing1 = first.K.inverse() * pixel1;
    Eigen::Vector3d bearing2 = second.K.inverse() * pixel2;

    if (!bearing1.allFinite() || !bearing2.allFinite() ||
        bearing1.norm() < config_.geometryEpsilon ||
        bearing2.norm() < config_.geometryEpsilon)
    {
        return fail(TriangulationFailure::InvalidCalibration);
    }

    bearing1.normalize();
    bearing2.normalize();

    // Rotate camera-frame bearings into world coordinates.
    Eigen::Vector3d d1 = Rcw1.transpose() * bearing1;
    Eigen::Vector3d d2 = Rcw2.transpose() * bearing2;

    d1.normalize();
    d2.normalize();

    const double cosine = std::clamp(d1.dot(d2), -1.0, 1.0);

    result.parallaxDegrees =
        std::acos(cosine) * 180.0 / std::acos(-1.0);

    if (!std::isfinite(result.parallaxDegrees) ||
        result.parallaxDegrees < config_.minParallaxDegrees)
    {
        return fail(TriangulationFailure::InsufficientParallax);
    }

    // Find the closest points on:
    // P1(s) = C1 + s*d1
    // P2(t) = C2 + t*d2
    const Eigen::Vector3d w0 = C1 - C2;

    const double b = d1.dot(d2);
    const double d = d1.dot(w0);
    const double e = d2.dot(w0);

    const double denominator = 1.0 - b * b;

    if (!std::isfinite(denominator) ||
        denominator <= config_.geometryEpsilon)
    {
        return fail(TriangulationFailure::DegenerateGeometry);
    }

    const double s = (b * e - d) / denominator;
    const double t = (e - b * d) / denominator;

    if (!std::isfinite(s) || !std::isfinite(t))
    {
        return fail(TriangulationFailure::DegenerateGeometry);
    }

    const Eigen::Vector3d pointOnRay1 = C1 + s * d1;
    const Eigen::Vector3d pointOnRay2 = C2 + t * d2;

    result.raySeparation =
        (pointOnRay1 - pointOnRay2).norm();

    if (!std::isfinite(result.raySeparation))
    {
        return fail(TriangulationFailure::DegenerateGeometry);
    }

    if (config_.maxRaySeparation > 0.0 &&
        result.raySeparation > config_.maxRaySeparation)
    {
        return fail(TriangulationFailure::ExcessiveRaySeparation);
    }

    // Midpoint of the closest points on the two viewing rays.
    result.pointWorld = 0.5 * (pointOnRay1 + pointOnRay2);

    if (!result.pointWorld.allFinite())
    {
        return fail(TriangulationFailure::DegenerateGeometry);
    }

    // Transform the estimate back into both camera coordinate systems.
    const Eigen::Vector3d pointCamera1 =
        Rcw1 * result.pointWorld + tcw1;

    const Eigen::Vector3d pointCamera2 =
        Rcw2 * result.pointWorld + tcw2;

    result.depthFirst = pointCamera1.z();
    result.depthSecond = pointCamera2.z();

    if (!std::isfinite(result.depthFirst) ||
        !std::isfinite(result.depthSecond) ||
        result.depthFirst <= config_.minDepth ||
        result.depthSecond <= config_.minDepth)
    {
        return fail(TriangulationFailure::NegativeDepth);
    }

    cv::Point2d projected1;
    cv::Point2d projected2;

    if (!projectPoint(pointCamera1, first.K, projected1) ||
        !projectPoint(pointCamera2, second.K, projected2))
    {
        return fail(TriangulationFailure::DegenerateGeometry);
    }

    result.reprojectionErrorFirst =
        pixelError(first.pixel, projected1);

    result.reprojectionErrorSecond =
        pixelError(second.pixel, projected2);

    if (!std::isfinite(result.reprojectionErrorFirst) ||
        !std::isfinite(result.reprojectionErrorSecond) ||
        result.reprojectionErrorFirst >
            config_.maxReprojectionErrorPixels ||
        result.reprojectionErrorSecond >
            config_.maxReprojectionErrorPixels)
    {
        return fail(TriangulationFailure::ExcessiveReprojectionError);
    }

    result.failure = TriangulationFailure::None;
    return result;
}