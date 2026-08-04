#include <cmath>
#include <chrono>
#include <iostream>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "rpg_polynomial_trajectory/minimum_snap_trajectories.h"
#include "rpg_polynomial_trajectory/polynomial_trajectory_helper.h"
#include "rpg_polynomial_trajectory/polynomial_trajectory_settings.h"

namespace {

bool IsFinite(const Eigen::Vector3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ring_boundary_regression");
  ros::Time::init();

  polynomial_trajectories::PolynomialTrajectorySettings settings;
  settings.minimization_weights = Eigen::VectorXd::Zero(5);
  settings.minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
  settings.polynomial_order = 11;
  settings.continuity_order = 4;

  constexpr int kNumPoints = 8;
  constexpr double kRadius = 5.0;
  constexpr double kHeight = 3.5;
  for (int i = 0; i < kNumPoints; ++i) {
    const double t = i * 2.0 * M_PI / kNumPoints;
    settings.way_points.emplace_back(kRadius * std::cos(t),
                                     kRadius * std::sin(t), kHeight);
  }

  Eigen::VectorXd segment_times(settings.way_points.size());
  segment_times.setOnes();

  constexpr double kMaxVelocity = 3.0;
  constexpr double kMaxNormalizedThrust = 15.0;
  constexpr double kMaxRollPitchRate = 2.0;
  constexpr double kSamplingFrequency = 50.0;

  const auto refinement_started = std::chrono::steady_clock::now();
  const auto refined =
      polynomial_trajectories::minimum_snap_trajectories::
          generateMinimumSnapRingTrajectoryWithSegmentRefinement(
              segment_times, settings);
  const auto refinement_finished = std::chrono::steady_clock::now();
  std::cout << "refinement_ms="
            << std::chrono::duration<double, std::milli>(
                   refinement_finished - refinement_started)
                   .count()
            << " T=" << refined.T.toSec() << std::endl;

  const auto enforcement_started = std::chrono::steady_clock::now();
  const auto constrained =
      polynomial_trajectories::minimum_snap_trajectories::
          implementation::enforceMaximumVelocityAndThrust(
              refined, settings, kMaxVelocity, kMaxNormalizedThrust,
              kMaxRollPitchRate);
  const auto enforcement_finished = std::chrono::steady_clock::now();
  std::cout << "enforcement_ms="
            << std::chrono::duration<double, std::milli>(
                   enforcement_finished - enforcement_started)
                   .count()
            << " T=" << constrained.T.toSec() << std::endl;

  const auto trajectory =
      trajectory_generation_helper::polynomials::samplePolynomial(
          constrained, kSamplingFrequency);

  if (trajectory.points.size() < 2) {
    std::cerr << "trajectory sampling returned fewer than 2 points\n";
    return 1;
  }

  const auto& first = trajectory.points.front();
  const auto& last = trajectory.points.back();
  const double closure_error = (last.position - first.position).norm();

  std::cout << "first=" << first.position.transpose() << '\n';
  std::cout << "last=" << last.position.transpose() << '\n';
  std::cout << "closure_error=" << closure_error << '\n';

  if (!IsFinite(first.position) || !IsFinite(last.position)) {
    std::cerr << "non-finite trajectory endpoint detected\n";
    return 1;
  }

  if (closure_error > 1e-6) {
    std::cerr << "ring endpoint failed to wrap back to the start state\n";
    return 1;
  }

  return 0;
}
