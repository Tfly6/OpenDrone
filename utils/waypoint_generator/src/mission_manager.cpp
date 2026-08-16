#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "waypoint_generator/mission_sequence.h"

namespace {

class MissionManager {
 public:
  MissionManager() : private_nh_("~") {
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
    private_nh_.param<std::string>("waypoint_topic", waypoint_topic_, "/waypoint_generator/waypoints");
    private_nh_.param<std::string>("goal_topic", goal_topic_, "/planning/mission_goal");
    private_nh_.param<std::string>("frame", default_frame_id_, "world");
    private_nh_.param<double>("timer_dt", timer_dt_, 0.05);
    private_nh_.param<double>("republish_dt", republish_dt_, 0.5);
    private_nh_.param<double>("goal_reached_distance", goal_reached_distance_, 0.4);
    private_nh_.param<double>("goal_dwell_time", goal_dwell_time_, 0.5);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MissionManager::odomCallback, this);
    path_sub_ = nh_.subscribe(waypoint_topic_, 1, &MissionManager::pathCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(timer_dt_), &MissionManager::timerCallback, this);

    ROS_INFO("[mission_manager] Ordered waypoint adapter: %s -> %s.",
             waypoint_topic_.c_str(), goal_topic_.c_str());
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher goal_pub_;
  ros::Timer timer_;

  nav_msgs::Odometry odom_;
  nav_msgs::Path mission_path_;
  waypoint_generator::MissionSequence sequence_;
  geometry_msgs::PoseStamped last_goal_;

  std::string odom_topic_;
  std::string waypoint_topic_;
  std::string goal_topic_;
  std::string default_frame_id_;
  double timer_dt_ = 0.05;
  double republish_dt_ = 0.5;
  double goal_reached_distance_ = 0.4;
  double goal_dwell_time_ = 0.5;

  bool has_odom_ = false;
  bool has_path_ = false;
  bool has_last_goal_ = false;
  ros::Time last_publish_time_;
  ros::Time goal_inside_since_;

  static bool isValidOrientation(const geometry_msgs::Quaternion& orientation) {
    const bool finite = std::isfinite(orientation.w) && std::isfinite(orientation.x) &&
                        std::isfinite(orientation.y) && std::isfinite(orientation.z);
    const double norm_squared = orientation.w * orientation.w + orientation.x * orientation.x +
                                orientation.y * orientation.y + orientation.z * orientation.z;
    return finite && norm_squared > 1e-6;
  }

  static double pointDistance(const geometry_msgs::Point& left, const geometry_msgs::Point& right) {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static bool samePose(const geometry_msgs::Pose& left, const geometry_msgs::Pose& right) {
    constexpr double kEpsilon = 1e-9;
    return pointDistance(left.position, right.position) <= kEpsilon &&
           std::abs(left.orientation.x - right.orientation.x) <= kEpsilon &&
           std::abs(left.orientation.y - right.orientation.y) <= kEpsilon &&
           std::abs(left.orientation.z - right.orientation.z) <= kEpsilon &&
           std::abs(left.orientation.w - right.orientation.w) <= kEpsilon;
  }

  bool sameMission(const nav_msgs::Path& candidate) const {
    if (!has_path_ || candidate.header.frame_id != mission_path_.header.frame_id ||
        candidate.poses.size() != mission_path_.poses.size()) {
      return false;
    }
    for (size_t index = 0; index < candidate.poses.size(); ++index) {
      if (!samePose(candidate.poses[index].pose, mission_path_.poses[index].pose)) {
        return false;
      }
    }
    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (sameMission(*msg)) {
      return;
    }

    mission_path_ = *msg;
    has_last_goal_ = false;
    goal_inside_since_ = ros::Time();
    sequence_.reset(mission_path_.poses.size());
    has_path_ = !sequence_.complete();

    if (!has_path_) {
      ROS_WARN("[mission_manager] Received empty path, mission cleared.");
      return;
    }

    ROS_INFO("[mission_manager] Loaded ordered mission with %zu waypoints; dispatching waypoint 1.",
             sequence_.waypointCount());
  }

  geometry_msgs::PoseStamped currentGoal(const ros::Time& now) const {
    geometry_msgs::PoseStamped goal;
    const geometry_msgs::PoseStamped& waypoint = mission_path_.poses[sequence_.activeIndex()];
    goal.header.seq = sequence_.commandSequence();
    goal.header.stamp = now;
    goal.header.frame_id = mission_path_.header.frame_id.empty()
                               ? default_frame_id_
                               : mission_path_.header.frame_id;
    goal.pose = waypoint.pose;
    if (!isValidOrientation(goal.pose.orientation)) {
      goal.pose.orientation.w = 1.0;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = 0.0;
    }
    return goal;
  }

  bool shouldPublishGoal(const geometry_msgs::PoseStamped& goal) const {
    if (!has_last_goal_ || goal.header.seq != last_goal_.header.seq) {
      return true;
    }
    return (goal.header.stamp - last_publish_time_).toSec() >= republish_dt_;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!has_odom_ || !has_path_ || sequence_.complete()) {
      return;
    }

    const ros::Time now = ros::Time::now();
    const geometry_msgs::Point& waypoint =
        mission_path_.poses[sequence_.activeIndex()].pose.position;
    if (pointDistance(odom_.pose.pose.position, waypoint) <= goal_reached_distance_) {
      if (goal_inside_since_.isZero()) {
        goal_inside_since_ = now;
      }
      if (goal_dwell_time_ <= 0.0 ||
          (now - goal_inside_since_).toSec() >= goal_dwell_time_) {
        const size_t completed_index = sequence_.activeIndex();
        goal_inside_since_ = ros::Time();
        has_last_goal_ = false;
        if (!sequence_.advance()) {
          ROS_INFO("[mission_manager] Mission complete after waypoint %zu.", completed_index + 1);
          return;
        }
        ROS_INFO("[mission_manager] Waypoint %zu reached; dispatching waypoint %zu.",
                 completed_index + 1, sequence_.activeIndex() + 1);
      }
    } else {
      goal_inside_since_ = ros::Time();
    }

    if (sequence_.complete()) {
      return;
    }
    const geometry_msgs::PoseStamped goal = currentGoal(now);
    if (!shouldPublishGoal(goal)) {
      return;
    }
    goal_pub_.publish(goal);
    last_goal_ = goal;
    has_last_goal_ = true;
    last_publish_time_ = now;
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_manager");
  MissionManager manager;
  ros::spin();
  return 0;
}
