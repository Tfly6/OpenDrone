#ifndef RPG_TRAJ_NODE_H
#define RPG_TRAJ_NODE_H

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <Eigen/Dense>

#include <math_utils/math_utils.h>

#include "rpg_polynomial_trajectory/polynomial_trajectory.h"
#include "rpg_polynomial_trajectory/polynomial_trajectory_helper.h"
#include "rpg_polynomial_trajectory/trajectory.h"

namespace polynomial_trajectories {

class RpgTrajNode {
 public:
  RpgTrajNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  double YawFromQuat(const geometry_msgs::Quaternion& q_msg) const {
    const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    return quaternion_to_rpy2(q).z();
  }

  geometry_msgs::Quaternion QuatFromYaw(const double yaw) const {
    const Eigen::Quaterniond q =
        quaternion_from_rpy(Eigen::Vector3d(0.0, 0.0, yaw));
    geometry_msgs::Quaternion q_msg;
    q_msg.w = q.w();
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    return q_msg;
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void waypointCallback(const nav_msgs::Path::ConstPtr& msg);
  void cancelCallback(const std_msgs::Empty::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);
  void planFromWaypoints(const nav_msgs::Path& path);

  polynomial_trajectories::TrajectoryPoint makeStartStateFromOdom(
      const nav_msgs::Odometry& odom) const;
  polynomial_trajectories::TrajectoryPoint makeEndStateFromPose(
      const geometry_msgs::PoseStamped& goal,
      double fallback_heading) const;

  Eigen::VectorXd buildInitialSegmentTimes(size_t num_segments) const;
  nav_msgs::Path toPath(const polynomial_trajectories::Trajectory& tr) const;
  trajectory_msgs::MultiDOFJointTrajectory toMultiDOF(
      const polynomial_trajectories::Trajectory& tr) const;
  trajectory_msgs::MultiDOFJointTrajectory toMultiDOFPoint(
      const polynomial_trajectories::TrajectoryPoint& p) const;
  bool getReferenceAtTime(const ros::Duration& t,
                          polynomial_trajectories::TrajectoryPoint* out) const;
  void publishTrajectoryProducts(const polynomial_trajectories::Trajectory& tr);
  void preemptAndActivateTrajectory(
      polynomial_trajectories::Trajectory&& new_traj);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber cancel_sub_;
  ros::Publisher path_pub_;
  ros::Publisher command_traj_pub_;
  ros::Timer timer_;

  std::mutex mtx_;
  bool has_odom_{false};
  bool has_traj_{false};
  bool has_pending_waypoints_{false};

  nav_msgs::Odometry odom_;
  nav_msgs::Path pending_waypoints_;
  polynomial_trajectories::Trajectory traj_;
  ros::Time traj_start_time_;

  std::atomic<uint64_t> traj_id_{0};
  uint64_t active_exec_id_{0};

  std::string odom_topic_;
  std::string goal_topic_;
  std::string waypoint_topic_;
  std::string cancel_topic_;
  std::string path_topic_;
  std::string command_traj_topic_;
  std::string world_frame_;

  bool use_minimum_snap_{true};
  bool use_segment_refine_{true};

  int order_of_continuity_{4};
  double max_velocity_{3.0};
  double max_normalized_thrust_{15.0};
  double max_roll_pitch_rate_{2.0};
  double sampling_frequency_{50.0};
  double exec_frequency_{50.0};
  double default_segment_time_{1.0};
  double default_goal_z_{1.5};
  bool split_trajectory_publish_{false};
};

}  // namespace polynomial_trajectories

#endif
