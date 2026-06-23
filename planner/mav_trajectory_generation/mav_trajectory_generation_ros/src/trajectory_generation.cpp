/**
 * ref : https://github.com/ethz-asl/mav_trajectory_generation/blob/master/mav_trajectory_generation_example/src/example_planner.cpp
 *
 * trajectory_generation
 *
 * @author TFly
 */

#include "mav_trajectory_generation_ros/trajectory_generation.h"

TrajectoryGeneration::TrajectoryGeneration(ros::NodeHandle& nh)
    : nh_(nh),
      max_v_(2.0),
      max_a_(2.0),
      use_nonlinear_opt_(false),
      nonlinear_max_iterations_(200),
      nonlinear_time_penalty_(500.0),
      dimension_(3),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {
  nh.param("max_v", max_v_, 2.0);
  nh.param("max_a", max_a_, 2.0);
  nh.param("use_nonlinear_opt", use_nonlinear_opt_, false);
  nh.param("nonlinear_max_iterations", nonlinear_max_iterations_, 200);
  nh.param("nonlinear_time_penalty", nonlinear_time_penalty_, 500.0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("/trajectory_generation/trajectory", 1);
  pub_trigger_ =
      nh.advertise<geometry_msgs::PoseStamped>("/waypoint_generator/traj_start_trigger", 0);

  sub_odom_ =
      nh.subscribe("/mavros/local_position/odom", 1, &TrajectoryGeneration::uavOdomCallback, this);
  sub_waypoint_ =
      nh.subscribe("/trajectory_generation/waypoint", 1, &TrajectoryGeneration::waypointCallback, this);
}

// Callback to get current Pose of UAV
void TrajectoryGeneration::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

void TrajectoryGeneration::waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
  waypoints_.clear();
  for (size_t i = 0; i < msg->poses.size(); ++i) {
    waypoints_.push_back(msg->poses[i].pose);
  }

  ROS_INFO("Received %zu waypoints.", waypoints_.size());
  if (!waypoints_.empty()) {
    planTrajectory();
  }
}

void TrajectoryGeneration::planTrajectory() {
  mav_trajectory_generation::Vertex::Vector vertices;
  mav_trajectory_generation::Vertex start(dimension_), end(dimension_);

  start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                      current_pose_.translation());
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      current_velocity_);

  vertices.push_back(start);
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    if (i == waypoints_.size() - 1) {
      end.makeStartOrEnd(
          Eigen::Vector3d(waypoints_[i].position.x,
                          waypoints_[i].position.y,
                          waypoints_[i].position.z),
          derivative_to_optimize_);
    } else {
      mav_trajectory_generation::Vertex middle(dimension_);
      middle.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          Eigen::Vector3d(waypoints_[i].position.x,
                          waypoints_[i].position.y,
                          waypoints_[i].position.z));
      vertices.push_back(middle);
    }
  }
  vertices.push_back(end);

  std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  mav_trajectory_generation::Trajectory trajectory;
  const int N = 10;

  if (use_nonlinear_opt_) {
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = nonlinear_max_iterations_;
    parameters.time_penalty = nonlinear_time_penalty_;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
      dimension_, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
    opt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
    opt.optimize();
    opt.getTrajectory(&trajectory);

    ROS_INFO("[TrajectoryGeneration] Using nonlinear optimization.");
  } else {
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
    opt.solveLinear();
    opt.getTrajectory(&trajectory);

    ROS_INFO("[TrajectoryGeneration] Using linear optimization.");
  }

  publishTrajectory(trajectory);
}

bool TrajectoryGeneration::publishTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory) {
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
  msg.header.frame_id = "map";
  pub_trajectory_.publish(msg);
  return true;
}

void TrajectoryGeneration::triggerWaypoints() {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  tf::poseEigenToMsg(current_pose_, pose.pose);
  pub_trigger_.publish(pose);
}
