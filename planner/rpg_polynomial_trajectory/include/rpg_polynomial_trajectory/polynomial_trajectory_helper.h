#pragma once

#include <rpg_polynomial_trajectory/polynomial_trajectory.h>
#include <rpg_polynomial_trajectory/polynomial_trajectory_settings.h>
#include <Eigen/Dense>

namespace trajectory_generation_helper {

namespace polynomials {
using Trajectory = polynomial_trajectories::Trajectory;
using TrajectoryPoint = polynomial_trajectories::TrajectoryPoint;


// Constrained Polynomials
Trajectory computeTimeOptimalTrajectory(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

Trajectory computeFixedTimeTrajectory(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double execution_time, const double sampling_frequency);

// Minimum Snap Style Polynomials
Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

// Sampling function
Trajectory samplePolynomial(
    const polynomial_trajectories::PolynomialTrajectory& polynomial,
    const double sampling_frequency);

}  // namespace polynomials

}  // namespace trajectory_generation_helper
