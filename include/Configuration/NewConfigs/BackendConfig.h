#pragma once

#include <string>

struct BackendConfig {
    // Monocular essential-matrix optimization.
    std::string essential_matrix_solver{"ceres"};
    std::string essential_matrix_solver_strategy{"levenberg_marquardt"};
    int monocular_em_ceres_max_num_iterations{30};
    double monocular_em_ceres_max_solver_time_seconds{0.05};
    double monocular_em_ceres_function_tolerance{1.0e-6};
    double monocular_em_ceres_gradient_tolerance{1.0e-10};
    double monocular_em_ceres_parameter_tolerance{1.0e-8};
    int monocular_em_ceres_num_threads{1};
    std::string monocular_em_ceres_linear_solver{"DENSE_QR"};

    // Monocular pose-only optimization.
    std::string visual_frontend_pose_only_solver{"ceres"};
    std::string visual_frontend_pose_only_solver_strategy{"levenberg_marquardt"};
    int monocular_pose_only_ceres_max_num_iterations{20};
    double monocular_pose_only_ceres_max_solver_time_seconds{0.05};
    double monocular_pose_only_ceres_function_tolerance{1.0e-6};
    double monocular_pose_only_ceres_gradient_tolerance{1.0e-10};
    double monocular_pose_only_ceres_parameter_tolerance{1.0e-8};
    int monocular_pose_only_ceres_num_threads{1};
    std::string monocular_pose_only_ceres_linear_solver{"DENSE_QR"};

    // Local bundle adjustment.
    std::string local_bundle_adjustment_solver{"ceres"};
    std::string local_bundle_adjustment_solver_strategy{"levenberg_marquardt"};
    int local_ba_ceres_max_num_iterations{20};
    double local_ba_ceres_max_solver_time_seconds{0.05};
    double local_ba_ceres_function_tolerance{1.0e-6};
    double local_ba_ceres_gradient_tolerance{1.0e-10};
    double local_ba_ceres_parameter_tolerance{1.0e-8};
    int local_ba_ceres_num_threads{1};
    std::string local_ba_ceres_linear_solver{"DENSE_SCHUR"};

    // Global bundle adjustment.
    std::string global_bundle_adjustment_solver{"ceres"};
    std::string global_bundle_adjustment_solver_strategy{"levenberg_marquardt"};
    int global_ba_ceres_max_num_iterations{50};
    double global_ba_ceres_max_solver_time_seconds{2.0};
    double global_ba_ceres_function_tolerance{1.0e-6};
    double global_ba_ceres_gradient_tolerance{1.0e-10};
    double global_ba_ceres_parameter_tolerance{1.0e-8};
    int global_ba_ceres_num_threads{4};
    std::string global_ba_ceres_linear_solver{"SPARSE_SCHUR"};
};
