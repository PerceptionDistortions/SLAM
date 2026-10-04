from engineering_ai.architecture.project_scanner import ProjectScanner
from engineering_ai.architecture.architecture_validator import (
    ArchitectureValidator
)


def main():
    project_root = "/home/anupam/slam_core"

    scanner = ProjectScanner(project_root)
    scan_result = scanner.scan()

    validator = ArchitectureValidator(scan_result)

    validator.validate()
    validator.print_report()


if __name__ == "__main__":
    main()

# Camera calibration validation	Intrinsics/distortion are correct
# IMU calibration & bias validation	IMU data/bias handling is sane
# Timestamp / synchronization validation	Camera–IMU temporal alignment is correct
# Feature detection validation	Features are sufficient and stable
# Feature matching validation	Correspondences are reliable
# RANSAC / geometric verification	Outliers are rejected
# Triangulation validation	3D landmarks are geometrically valid
# Pose estimation validation	Estimated motion is physically/geometrically consistent
# Tracking quality validation	Tracking doesn't silently degrade
# State-estimator consistency	EKF/ESEKF covariance and innovations make sense
# Trajectory accuracy validation	SLAM trajectory is actually correct
# Map quality validation	Landmarks/map are geometrically meaningful
# Loop-closure validation	False loops don't corrupt the map
# Failure/degradation validation	System behaves sensibly under bad input
# Runtime/performance validation	Meets real-time XR constraints