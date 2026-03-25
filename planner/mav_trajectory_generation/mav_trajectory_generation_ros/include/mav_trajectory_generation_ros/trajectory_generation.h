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
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include "mav_trajectory_generation_ros/ros_conversions.h"
#include "mav_trajectory_generation/trajectory.h"


class TrajectoryGeneration {
 public:
  TrajectoryGeneration(ros::NodeHandle& nh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void waypointCallback(const nav_msgs::Path::ConstPtr& msg);

//   void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  // bool planTrajectory(const Eigen::VectorXd& goal_pos,
  //                     const Eigen::VectorXd& goal_vel,
  //                     mav_trajectory_generation::Trajectory* trajectory);
  void planTrajectory();
  void checkAndReplan(const ros::TimerEvent&);

//   bool planTrajectory(const Eigen::VectorXd& goal_pos,
//                       const Eigen::VectorXd& goal_vel,
//                       const Eigen::VectorXd& start_pos,
//                       const Eigen::VectorXd& start_vel,
//                       double v_max, double a_max,
//                       mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

  void triggerWaypoints();

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_trigger_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_waypoint_;
  ros::Timer replan_timer_;

  ros::NodeHandle& nh_;
  
  std::vector<geometry_msgs::Pose> waypoints_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
  const int dimension_;
  const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;

  // 循环重规划相关
  bool loop_trajectory_ = false;    // 是否开启循环模式
  double trajectory_duration_ = 0.0; // 当前轨迹总时长
  ros::Time trajectory_start_time_;  // 轨迹开始执行的时间
  double replan_ratio_ = 0.8;        // 剩余比例触发重规划（默认80%时触发）
  bool trajectory_active_ = false;   // 当前是否有轨迹在执行

};
#endif