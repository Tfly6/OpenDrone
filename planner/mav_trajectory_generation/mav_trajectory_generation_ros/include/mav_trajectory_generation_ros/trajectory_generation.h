/**
 * ref : https://github.com/ethz-asl/mav_trajectory_generation/blob/master/mav_trajectory_generation_example/include/mav_trajectory_generation_example/example_planner.h
 *
 * trajectory_generation
 *
 * @author TFly
 */

#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen_conversions/eigen_msg.h>
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation_ros/ros_conversions.h"
#include "mav_trajectory_generation/trajectory.h"

class TrajectoryGeneration {
 public:
  TrajectoryGeneration(ros::NodeHandle& nh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);
  void waypointCallback(const nav_msgs::Path::ConstPtr& msg);

  void planTrajectory();
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
  void triggerWaypoints();

 private:
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_trigger_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_waypoint_;

  ros::NodeHandle& nh_;

  std::vector<geometry_msgs::Pose> waypoints_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_;  // m/s
  double max_a_;  // m/s^2
  double max_ang_v_;
  double max_ang_a_;

  bool use_nonlinear_opt_;
  int nonlinear_max_iterations_;
  double nonlinear_time_penalty_;

  const int dimension_;
  const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;
};

#endif
