#include <ros/ros.h>
#include <Eigen/Dense>

#include "rpg_polynomial_trajectory/polynomial_trajectory_helper.h"
#include "rpg_polynomial_trajectory/polynomial_trajectory.h"
#include "rpg_polynomial_trajectory/trajectory.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rpg_poly_test");
  ros::NodeHandle nh;
  using namespace polynomial_trajectories;
  using namespace trajectory_generation_helper::polynomials;

  TrajectoryPoint s0;
  s0.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  s0.velocity = Eigen::Vector3d::Zero();
  s0.acceleration = Eigen::Vector3d::Zero();
  s0.jerk = Eigen::Vector3d::Zero();
  s0.heading = 0.0;

  TrajectoryPoint s1;
  s1.position = Eigen::Vector3d(5.0, 0.0, 1.5);
  s1.velocity = Eigen::Vector3d::Zero();
  s1.acceleration = Eigen::Vector3d::Zero();
  s1.jerk = Eigen::Vector3d::Zero();
  s1.heading = 0.0;

  const int order_of_continuity = 4;  // pos vel acc jerk
  const double max_velocity = 3.0;
  const double max_normalized_thrust = 15.0;
  const double max_roll_pitch_rate = 2.0;
  const double sampling_frequency = 50.0;

  Trajectory traj = computeTimeOptimalTrajectory(
      s0, s1, order_of_continuity,
      max_velocity, max_normalized_thrust,
      max_roll_pitch_rate, sampling_frequency);

  ROS_INFO("Generated trajectory with %zu points", traj.points.size());

  if (!traj.points.empty()) {
    const auto& first = traj.points.front();
    const auto& last  = traj.points.back();

    ROS_INFO_STREAM("first pos: " << first.position.transpose());
    ROS_INFO_STREAM("last  pos: " << last.position.transpose());
    ROS_INFO_STREAM("last  heading: " << last.heading);
  }

  return 0;
}