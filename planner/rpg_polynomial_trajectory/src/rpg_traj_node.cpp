#include "rpg_polynomial_trajectory/rpg_traj_node.h"

#include <cmath>
#include <utility>

namespace polynomial_trajectories  {

RpgTrajNode::RpgTrajNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("odom_topic", odom_topic_,
                          "/mavros/local_position/odom");
  pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
  pnh_.param<std::string>("waypoint_topic", waypoint_topic_,
                          "/rpg_traj/waypoints");
  pnh_.param<std::string>("cancel_topic", cancel_topic_, "/rpg_traj/cancel");

  pnh_.param<std::string>("path_topic", path_topic_, "/rpg_traj/path");
  pnh_.param<std::string>("command_traj_topic", command_traj_topic_,
                          "/command/trajectory");
  pnh_.param<std::string>("world_frame", world_frame_, "map");

  pnh_.param("use_minimum_snap", use_minimum_snap_, true);
  pnh_.param("use_segment_refine", use_segment_refine_, true);

  pnh_.param("order_of_continuity", order_of_continuity_, 4);
  pnh_.param("max_velocity", max_velocity_, 3.0);
  pnh_.param("max_normalized_thrust", max_normalized_thrust_, 15.0);
  pnh_.param("max_roll_pitch_rate", max_roll_pitch_rate_, 2.0);
  pnh_.param("sampling_frequency", sampling_frequency_, 50.0);
  pnh_.param("exec_frequency", exec_frequency_, 50.0);
  pnh_.param("default_segment_time", default_segment_time_, 1.0);
  pnh_.param("default_goal_z", default_goal_z_, 1.5);
  pnh_.param("split_trajectory_publish", split_trajectory_publish_, false);

  odom_sub_ = nh_.subscribe(odom_topic_, 1, &RpgTrajNode::odomCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &RpgTrajNode::goalCallback, this);
  waypoint_sub_ =
      nh_.subscribe(waypoint_topic_, 1, &RpgTrajNode::waypointCallback, this);
  cancel_sub_ =
      nh_.subscribe(cancel_topic_, 1, &RpgTrajNode::cancelCallback, this);

  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  command_traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      command_traj_topic_, 1, true);

  timer_ = nh_.createTimer(ros::Duration(1.0 / exec_frequency_),
                           &RpgTrajNode::timerCallback, this);

  ROS_INFO("[rpg_traj_node] started.");
}

polynomial_trajectories::TrajectoryPoint RpgTrajNode::makeStartStateFromOdom(
    const nav_msgs::Odometry& odom) const {
  polynomial_trajectories::TrajectoryPoint s;
  s.time_from_start = ros::Duration(0.0);

  s.position = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
                               odom.pose.pose.position.z);

  s.orientation = Eigen::Quaterniond(
      odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

  Eigen::Vector3d velocity_body = Eigen::Vector3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                               odom.twist.twist.linear.z);
  s.velocity = s.orientation * velocity_body;

  s.acceleration = Eigen::Vector3d::Zero();
  s.jerk = Eigen::Vector3d::Zero();
  s.snap = Eigen::Vector3d::Zero();

  s.bodyrates = Eigen::Vector3d(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                               odom.twist.twist.angular.z);

  s.angular_acceleration = Eigen::Vector3d::Zero();
  s.angular_jerk = Eigen::Vector3d::Zero();
  s.angular_snap = Eigen::Vector3d::Zero();

  s.heading = YawFromQuat(odom.pose.pose.orientation);
  s.heading_rate = 0.0;
  s.heading_acceleration = 0.0;
  return s;
}

polynomial_trajectories::TrajectoryPoint RpgTrajNode::makeEndStateFromPose(
    const geometry_msgs::PoseStamped& goal, double fallback_heading) const {
  polynomial_trajectories::TrajectoryPoint s;
  s.time_from_start = ros::Duration(0.0);

  s.position = Eigen::Vector3d(goal.pose.position.x, goal.pose.position.y,
                               goal.pose.position.z);

  if (std::abs(s.position.z()) < 1e-6) {
    s.position.z() = default_goal_z_;
  }

  s.velocity = Eigen::Vector3d::Zero();
  s.acceleration = Eigen::Vector3d::Zero();
  s.jerk = Eigen::Vector3d::Zero();
  s.snap = Eigen::Vector3d::Zero();

  s.bodyrates = Eigen::Vector3d::Zero();
  s.angular_acceleration = Eigen::Vector3d::Zero();
  s.angular_jerk = Eigen::Vector3d::Zero();
  s.angular_snap = Eigen::Vector3d::Zero();

  const double q_norm = std::sqrt(
      goal.pose.orientation.x * goal.pose.orientation.x +
      goal.pose.orientation.y * goal.pose.orientation.y +
      goal.pose.orientation.z * goal.pose.orientation.z +
      goal.pose.orientation.w * goal.pose.orientation.w);

  if (q_norm > 1e-6) {
    s.heading = YawFromQuat(goal.pose.orientation);
  } else {
    s.heading = fallback_heading;
  }

  s.heading_rate = 0.0;
  s.heading_acceleration = 0.0;
  s.orientation = Eigen::Quaterniond::Identity();
  return s;
}

Eigen::VectorXd RpgTrajNode::buildInitialSegmentTimes(size_t num_segments) const {
  Eigen::VectorXd seg_times(num_segments);
  seg_times.setConstant(default_segment_time_);
  return seg_times;
}

nav_msgs::Path RpgTrajNode::toPath(
    const polynomial_trajectories::Trajectory& tr) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = world_frame_;

  for (const auto& p : tr.points) {
    geometry_msgs::PoseStamped ps;
    ps.header = path_msg.header;
    ps.header.stamp = traj_start_time_ + p.time_from_start;
    ps.pose.position.x = p.position.x();
    ps.pose.position.y = p.position.y();
    ps.pose.position.z = p.position.z();
    ps.pose.orientation = QuatFromYaw(p.heading);
    path_msg.poses.push_back(ps);
  }
  return path_msg;
}

trajectory_msgs::MultiDOFJointTrajectory RpgTrajNode::toMultiDOF(
    const polynomial_trajectories::Trajectory& tr) const {
  trajectory_msgs::MultiDOFJointTrajectory msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = world_frame_;
  msg.joint_names.push_back("base_link");

  for (const auto& p : tr.points) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt;

    geometry_msgs::Transform tfm;
    tfm.translation.x = p.position.x();
    tfm.translation.y = p.position.y();
    tfm.translation.z = p.position.z();
    tfm.rotation = QuatFromYaw(p.heading);
    pt.transforms.push_back(tfm);

    geometry_msgs::Twist vel;
    vel.linear.x = p.velocity.x();
    vel.linear.y = p.velocity.y();
    vel.linear.z = p.velocity.z();
    vel.angular.z = p.heading_rate;
    pt.velocities.push_back(vel);

    geometry_msgs::Twist acc;
    acc.linear.x = p.acceleration.x();
    acc.linear.y = p.acceleration.y();
    acc.linear.z = p.acceleration.z();
    acc.angular.z = p.heading_acceleration;
    pt.accelerations.push_back(acc);

    pt.time_from_start = p.time_from_start;
    msg.points.push_back(pt);
  }

  return msg;
}

trajectory_msgs::MultiDOFJointTrajectory RpgTrajNode::toMultiDOFPoint(
    const polynomial_trajectories::TrajectoryPoint& p) const {
  trajectory_msgs::MultiDOFJointTrajectory msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = world_frame_;
  msg.joint_names.push_back("base_link");

  trajectory_msgs::MultiDOFJointTrajectoryPoint pt;

  geometry_msgs::Transform tfm;
  tfm.translation.x = p.position.x();
  tfm.translation.y = p.position.y();
  tfm.translation.z = p.position.z();
  tfm.rotation = QuatFromYaw(p.heading);
  pt.transforms.push_back(tfm);

  geometry_msgs::Twist vel;
  vel.linear.x = p.velocity.x();
  vel.linear.y = p.velocity.y();
  vel.linear.z = p.velocity.z();
  vel.angular.z = p.heading_rate;
  pt.velocities.push_back(vel);

  geometry_msgs::Twist acc;
  acc.linear.x = p.acceleration.x();
  acc.linear.y = p.acceleration.y();
  acc.linear.z = p.acceleration.z();
  acc.angular.z = p.heading_acceleration;
  pt.accelerations.push_back(acc);

  pt.time_from_start = p.time_from_start;
  msg.points.push_back(pt);
  return msg;
}

bool RpgTrajNode::getReferenceAtTime(
    const ros::Duration& t,
    polynomial_trajectories::TrajectoryPoint* out) const {
  if (!out || traj_.points.empty()) {
    return false;
  }

  for (const auto& p : traj_.points) {
    if (p.time_from_start >= t) {
      *out = p;
      return true;
    }
  }

  *out = traj_.points.back();
  return true;
}

void RpgTrajNode::publishTrajectoryProducts(
    const polynomial_trajectories::Trajectory& tr) {
  path_pub_.publish(toPath(tr));

  if (split_trajectory_publish_) {
    if (!tr.points.empty()) {
      command_traj_pub_.publish(toMultiDOFPoint(tr.points.front()));
    }
    return;
  }

  command_traj_pub_.publish(toMultiDOF(tr));
}

void RpgTrajNode::preemptAndActivateTrajectory(
    polynomial_trajectories::Trajectory&& new_traj) {
  const uint64_t new_id = ++traj_id_;
  active_exec_id_ = new_id;

  traj_ = std::move(new_traj);
  traj_start_time_ = ros::Time::now();
  has_traj_ = !traj_.points.empty();

  if (has_traj_) {
    publishTrajectoryProducts(traj_);
    ROS_INFO("[rpg_traj_node] active trajectory id=%lu, points=%zu",
             static_cast<unsigned long>(new_id), traj_.points.size());
  } else {
    ROS_WARN("[rpg_traj_node] generated empty trajectory.");
  }
}

void RpgTrajNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  odom_ = *msg;
  has_odom_ = true;
}

void RpgTrajNode::cancelCallback(const std_msgs::Empty::ConstPtr& msg) {
  (void)msg;
  std::lock_guard<std::mutex> lock(mtx_);
  has_traj_ = false;
  traj_.points.clear();
  ROS_WARN("[rpg_traj_node] trajectory canceled.");
}

void RpgTrajNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!has_odom_) {
    ROS_WARN("[rpg_traj_node] goal received but odom not ready.");
    return;
  }

  const auto s0 = makeStartStateFromOdom(odom_);
  const auto s1 = makeEndStateFromPose(*msg, s0.heading);

  try {
    auto tr = trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
        s0, s1, order_of_continuity_, max_velocity_, max_normalized_thrust_,
        max_roll_pitch_rate_, sampling_frequency_);

    preemptAndActivateTrajectory(std::move(tr));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[rpg_traj_node] goal planning exception: " << e.what());
  } catch (...) {
    ROS_ERROR("[rpg_traj_node] goal planning unknown exception.");
  }
}

void RpgTrajNode::waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!has_odom_) {
    ROS_WARN("[rpg_traj_node] waypoints received but odom not ready.");
    return;
  }
  if (msg->poses.empty()) {
    ROS_WARN("[rpg_traj_node] empty waypoint path.");
    return;
  }

  const auto start = makeStartStateFromOdom(odom_);

  auto end = start;
  end.position = Eigen::Vector3d(msg->poses.back().pose.position.x,
                                 msg->poses.back().pose.position.y,
                                 msg->poses.back().pose.position.z);
  if (std::abs(end.position.z()) < 1e-6) {
    end.position.z() = default_goal_z_;
  }
  end.velocity = Eigen::Vector3d::Zero();
  end.acceleration = Eigen::Vector3d::Zero();
  end.jerk = Eigen::Vector3d::Zero();
  end.snap = Eigen::Vector3d::Zero();

  try {
    polynomial_trajectories::Trajectory tr;

    if (!use_minimum_snap_ || msg->poses.size() < 2) {
      tr = trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
          start, end, order_of_continuity_, max_velocity_,
          max_normalized_thrust_, max_roll_pitch_rate_, sampling_frequency_);
    } else {
      polynomial_trajectories::PolynomialTrajectorySettings settings;
      settings.way_points.clear();

      for (size_t i = 0; i + 1 < msg->poses.size(); ++i) {
        settings.way_points.emplace_back(msg->poses[i].pose.position.x,
                                         msg->poses[i].pose.position.y,
                                         msg->poses[i].pose.position.z);
      }

      const Eigen::VectorXd seg_times =
          buildInitialSegmentTimes(msg->poses.size());

      if (use_segment_refine_) {
        tr = trajectory_generation_helper::polynomials::
            generateMinimumSnapTrajectoryWithSegmentRefinement(
                seg_times, start, end, settings, max_velocity_,
                max_normalized_thrust_, max_roll_pitch_rate_,
                sampling_frequency_);
      } else {
        tr = trajectory_generation_helper::polynomials::
            generateMinimumSnapTrajectory(seg_times, start, end, settings,
                                          max_velocity_, max_normalized_thrust_,
                                          max_roll_pitch_rate_,
                                          sampling_frequency_);
      }
    }

    preemptAndActivateTrajectory(std::move(tr));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[rpg_traj_node] waypoint planning exception: " << e.what());
  } catch (...) {
    ROS_ERROR("[rpg_traj_node] waypoint planning unknown exception.");
  }
}

void RpgTrajNode::timerCallback(const ros::TimerEvent& event) {
  (void)event;
  std::lock_guard<std::mutex> lock(mtx_);
  if (!has_traj_) {
    return;
  }

  const uint64_t exec_id = active_exec_id_;
  polynomial_trajectories::TrajectoryPoint ref;
  const ros::Duration t = ros::Time::now() - traj_start_time_;

  if (!getReferenceAtTime(t, &ref)) {
    return;
  }
  if (exec_id != active_exec_id_) {
    return;
  }

  if (split_trajectory_publish_) {
    command_traj_pub_.publish(toMultiDOFPoint(ref));
  }
}
}  // namespace polynomial_trajectories

int main(int argc, char** argv) {
  ros::init(argc, argv, "rpg_traj_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  polynomial_trajectories::RpgTrajNode node(nh, pnh);
  ros::spin();
  return 0;
}
