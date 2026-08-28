#include <algorithm>
#include <string>
#include <vector>

#include <boost/format.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include "sample_waypoints.h"

using bfmt = boost::format;

namespace {

ros::Publisher pub_waypoints;
ros::Publisher pub_auto_trigger;
ros::Publisher pub_vis;
ros::NodeHandle* private_nh = nullptr;

std::string waypoint_type("manual");
enum class WaypointMode {
  kManualPreset,
  kCirclePreset,
  kEightPreset,
  kPointPreset,
};

WaypointMode waypoint_mode = WaypointMode::kManualPreset;
std::string frame_id("map");
bool auto_trigger = false;
bool start_from_current_position = true;
bool is_odom_ready = false;
bool visualize = false;
int num_points = 30;
int closure_points = 2;
nav_msgs::Odometry odom;
nav_msgs::Path waypoints;

const geometry_msgs::Pose* GetStartPoseForPreset() {
  if (start_from_current_position && is_odom_ready) {
    return &odom.pose.pose;
  }
  return nullptr;
}
WaypointMode ParseWaypointMode(const std::string& type) {
  if (type == "manual") {
    return WaypointMode::kManualPreset;
  }
  if (type == "circle") {
    return WaypointMode::kCirclePreset;
  }
  if (type == "eight") {
    return WaypointMode::kEightPreset;
  }
  if (type == "point") {
    return WaypointMode::kPointPreset;
  }
  ROS_WARN("[waypoint_generator] Unknown waypoint_type '%s', fallback to manual.",
           type.c_str());
  return WaypointMode::kManualPreset;
}

bool IsPresetMode(WaypointMode mode) {
  return mode == WaypointMode::kManualPreset ||
         mode == WaypointMode::kCirclePreset ||
         mode == WaypointMode::kEightPreset ||
         mode == WaypointMode::kPointPreset;
}

ros::NodeHandle& PrivateNh() {
  ROS_ASSERT(private_nh != nullptr);
  return *private_nh;
}

void PublishWaypoints() {
  waypoints.header.frame_id = frame_id;
  waypoints.header.stamp = ros::Time::now();
  pub_waypoints.publish(waypoints);
  waypoints.poses.clear();
}

void PublishWaypointsVis() {
  if (!visualize) {
    return;
  }

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = frame_id;
  pose_array.header.stamp = ros::Time::now();

  if (is_odom_ready) {
    pose_array.poses.push_back(odom.pose.pose);
  }

  for (const auto& waypoint : waypoints.poses) {
    pose_array.poses.push_back(waypoint.pose);
  }

  pub_vis.publish(pose_array);
}

void PublishPresetWaypoints(const nav_msgs::Path& path) {
  waypoints = path;
  for (std::size_t index = 0; index < waypoints.poses.size(); ++index) {
    waypoints.poses[index].header.seq = static_cast<uint32_t>(index);
  }
  PublishWaypointsVis();
  PublishWaypoints();
}

bool GetWaypointCount(ros::NodeHandle& nh, int* waypoint_count,
                      std::string* waypoint_prefix) {
  if (nh.getParam("waypoint_num", *waypoint_count)) {
    *waypoint_prefix = "waypoint";
    return true;
  }

  return false;
}

nav_msgs::Path LoadManualWaypoints(ros::NodeHandle& nh) {
  int waypoint_count = 0;
  std::string waypoint_prefix;
  ROS_ASSERT_MSG(GetWaypointCount(nh, &waypoint_count, &waypoint_prefix),
                 "Missing '~waypoint_num' parameter.");
  ROS_ASSERT_MSG(waypoint_count > 0, "waypoint_num must be > 0.");

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id;

  for (int i = 0; i < waypoint_count; ++i) {
    const std::string name = boost::str(bfmt("%s%d") % waypoint_prefix % i);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    ROS_ASSERT(nh.getParam(name + "_x", x));
    ROS_ASSERT(nh.getParam(name + "_y", y));
    ROS_ASSERT(nh.getParam(name + "_z", z));

    geometry_msgs::PoseStamped pt;
    pt.header.frame_id = frame_id;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    pt.pose.position.x = x;
    pt.pose.position.y = y;
    pt.pose.position.z = z;
    path_msg.poses.push_back(pt);
  }

  ROS_INFO("Loaded %zu manual waypoints from parameters.", path_msg.poses.size());
  return path_msg;
}

bool HandlePresetTrigger(const ros::Time& trigger_time, ros::NodeHandle& nh) {
  if (!IsPresetMode(waypoint_mode)) {
    return false;
  }

  (void)trigger_time;
  if (start_from_current_position && !is_odom_ready) {
    ROS_WARN("[waypoint_generator] Waiting for odom before generating preset waypoints.");
    return false;
  }
  ROS_INFO_STREAM("Pattern " << waypoint_type << " generated!");

  switch (waypoint_mode) {
    case WaypointMode::kManualPreset:
      PublishPresetWaypoints(LoadManualWaypoints(nh));
      break;
    case WaypointMode::kCirclePreset:
      PublishPresetWaypoints(circle(num_points, closure_points, GetStartPoseForPreset()));
      break;
    case WaypointMode::kEightPreset:
      PublishPresetWaypoints(eight(num_points, closure_points, GetStartPoseForPreset()));
      break;
    case WaypointMode::kPointPreset:
      PublishPresetWaypoints(point(GetStartPoseForPreset()));
      break;
    default:
      return false;
  }

  return true;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  is_odom_ready = true;
  odom = *msg;

  if (auto_trigger) {
    geometry_msgs::PoseStamped trigger_msg;
    trigger_msg.header.stamp = ros::Time::now();
    trigger_msg.header.frame_id = frame_id;
    trigger_msg.pose.position.z = 0.5;
    pub_auto_trigger.publish(trigger_msg);
    auto_trigger = false;
  }

}

// void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//   (void)msg;
//   if (HandlePresetTrigger(ros::Time::now(), PrivateNh())) {
//     return;
//   }
// }

void TrajStartTriggerCallback(const geometry_msgs::PoseStamped&) {
  ROS_WARN("[waypoint_generator] Trigger!");

  HandlePresetTrigger(ros::Time::now(), PrivateNh());
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_generator");
  ros::NodeHandle private_nh_local("~");
  private_nh = &private_nh_local;

  private_nh_local.param<std::string>("waypoint_type", waypoint_type, "manual");
  waypoint_mode = ParseWaypointMode(waypoint_type);
  private_nh_local.param<std::string>("frame", frame_id, "map");
  private_nh_local.param<bool>("autoTrigger", auto_trigger, false);
  private_nh_local.param<bool>("startFromCurrentPosition", start_from_current_position, true);
  private_nh_local.param<bool>("visualize", visualize, false);
  private_nh_local.param<int>("num_points", num_points, 30);
  private_nh_local.param<int>("closure_points", closure_points, 2);

  num_points = std::max(3, num_points);
  closure_points = std::max(0, closure_points);
  if (closure_points > num_points) {
    ROS_WARN("[waypoint_generator] closure_points (%d) > num_points (%d), clamping.",
             closure_points, num_points);
    closure_points = num_points;
  }

  ros::Subscriber odom_sub =
      private_nh_local.subscribe("odom", 10, OdomCallback);
//   ros::Subscriber goal_sub =
//       private_nh_local.subscribe("goal", 10, GoalCallback);
  ros::Subscriber traj_sub =
      private_nh_local.subscribe("traj_start_trigger", 10, TrajStartTriggerCallback);

  pub_auto_trigger =
      private_nh_local.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);
  pub_waypoints =
      private_nh_local.advertise<nav_msgs::Path>("waypoints", 50, true);
  pub_vis =
      private_nh_local.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

  ros::spin();
  return 0;
}
