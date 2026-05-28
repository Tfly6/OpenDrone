#include "rpg_polynomial_trajectory/polynomial_trajectory_helper.h"

#include <rpg_polynomial_trajectory/constrained_polynomial_trajectories.h>
#include <rpg_polynomial_trajectory/minimum_snap_trajectories.h>
#include <rpg_polynomial_trajectory/polynomial_trajectories_common.h>

namespace trajectory_generation_helper {

namespace polynomials {

// Constrained Polynomials
polynomial_trajectories::Trajectory computeTimeOptimalTrajectory(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::constrained_polynomial_trajectories::
          computeTimeOptimalTrajectory(s0, s1, order_of_continuity,
                                       max_velocity, max_normalized_thrust,
                                       max_roll_pitch_rate);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory computeFixedTimeTrajectory(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double execution_time, const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::constrained_polynomial_trajectories::
          computeFixedTimeTrajectory(s0, s1, order_of_continuity,
                                     execution_time);

  return samplePolynomial(polynomial, sampling_frequency);
}

// Minimum Snap Style Polynomials
Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapTrajectory(segment_times, start_state, end_state,
                                        trajectory_settings);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapTrajectory(initial_segment_times, start_state,
                                        end_state, trajectory_settings,
                                        max_velocity, max_normalized_thrust,
                                        max_roll_pitch_rate);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapTrajectoryWithSegmentRefinement(
              initial_segment_times, start_state, end_state,
              trajectory_settings);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const TrajectoryPoint& start_state,
    const TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapTrajectoryWithSegmentRefinement(
              initial_segment_times, start_state, end_state,
              trajectory_settings, max_velocity, max_normalized_thrust,
              max_roll_pitch_rate);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapRingTrajectory(segment_times, trajectory_settings);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapRingTrajectory(
              initial_segment_times, trajectory_settings, max_velocity,
              max_normalized_thrust, max_roll_pitch_rate);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapRingTrajectoryWithSegmentRefinement(
              initial_segment_times, trajectory_settings);

  return samplePolynomial(polynomial, sampling_frequency);
}

Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency) {
  polynomial_trajectories::PolynomialTrajectory polynomial =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapRingTrajectoryWithSegmentRefinement(
              initial_segment_times, trajectory_settings, max_velocity,
              max_normalized_thrust, max_roll_pitch_rate);

  return samplePolynomial(polynomial, sampling_frequency);
}

// Sampling function
Trajectory samplePolynomial(
    const polynomial_trajectories::PolynomialTrajectory& polynomial,
    const double sampling_frequency) {
  if (polynomial.trajectory_type ==
      polynomial_trajectories::TrajectoryType::UNDEFINED) {
    return Trajectory();
  }

  Trajectory trajectory;

  trajectory.points.push_back(polynomial.start_state);

  const ros::Duration dt(1.0 / sampling_frequency);
  ros::Duration time_from_start = polynomial.start_state.time_from_start + dt;

  while (time_from_start < polynomial.T) {
    trajectory.points.push_back(polynomial_trajectories::getPointFromTrajectory(
        polynomial, time_from_start));
    time_from_start += dt;
  }

  trajectory.points.push_back(polynomial.end_state);

  trajectory.trajectory_type =
      polynomial_trajectories::Trajectory::TrajectoryType::GENERAL;

  return trajectory;
}

}  // namespace polynomials

}  // namespace trajectory_generation_helper
