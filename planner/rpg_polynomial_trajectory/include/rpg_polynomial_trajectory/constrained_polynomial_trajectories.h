#pragma once

#include <Eigen/Dense>

#include "rpg_polynomial_trajectory/polynomial_trajectory.h"

namespace polynomial_trajectories {

namespace constrained_polynomial_trajectories {

// order_of_continuity: 1 = position, 2 = velocity, ....

PolynomialTrajectory computeTimeOptimalTrajectory(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate);

PolynomialTrajectory computeFixedTimeTrajectory(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double execution_time);

// these functions should not be used from the outside
namespace implementation {
Eigen::MatrixXd computeConstraintMatriceA(const int order_of_continuity,
                                          const double t);

Eigen::VectorXd computeConstraintMatriceB(
    const int order_of_continuity,
    const TrajectoryPoint& state, const int axis);

std::vector<Eigen::MatrixXd> computeTrajectoryCoeff(
    const TrajectoryPoint& s0,
    const TrajectoryPoint& s1, const int order_of_continuity,
    const double T);

Eigen::Vector3d computeMaximaGradient(const PolynomialTrajectory& trajectory,
                                      const Eigen::Vector3d& maxima,
                                      const double order_of_continuity);
}  // namespace implementation

}  // namespace constrained_polynomial_trajectories

}  // namespace polynomial_trajectories
