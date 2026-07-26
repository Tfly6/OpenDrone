#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace {

struct SampledPoint {
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  double progress = 0.0;
  size_t segment_index = 0;
};

class MissionManager {
 public:
  MissionManager() : private_nh_("~") {
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
    private_nh_.param<std::string>("waypoint_topic", waypoint_topic_, "/waypoint_generator/waypoints");
    private_nh_.param<std::string>("goal_topic", goal_topic_, "/planning/mission_goal");
    private_nh_.param<std::string>("frame", default_frame_id_, "world");
    private_nh_.param<double>("timer_dt", timer_dt_, 0.05);
    private_nh_.param<double>("republish_dt", republish_dt_, 0.5);
    private_nh_.param<double>("lookahead_distance", lookahead_distance_, 2.0);
    private_nh_.param<double>("goal_reached_distance", goal_reached_distance_, 0.4);
    private_nh_.param<double>("goal_update_distance", goal_update_distance_, 0.35);
    private_nh_.param<double>("goal_update_yaw", goal_update_yaw_, 0.26);
    private_nh_.param<bool>("use_final_waypoint_orientation", use_final_waypoint_orientation_, true);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MissionManager::odomCallback, this);
    path_sub_ = nh_.subscribe(waypoint_topic_, 1, &MissionManager::pathCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(timer_dt_), &MissionManager::timerCallback, this);

    ROS_INFO("[mission_manager] Listening on %s, publishing goals to %s.",
             waypoint_topic_.c_str(),
             goal_topic_.c_str());
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
  std::vector<double> cumulative_lengths_;
  geometry_msgs::PoseStamped last_goal_;

  std::string odom_topic_;
  std::string waypoint_topic_;
  std::string goal_topic_;
  std::string default_frame_id_;

  double timer_dt_ = 0.05;
  double republish_dt_ = 0.5;
  double lookahead_distance_ = 2.0;
  double goal_reached_distance_ = 0.4;
  double goal_update_distance_ = 0.35;
  double goal_update_yaw_ = 0.26;
  bool use_final_waypoint_orientation_ = true;

  bool has_odom_ = false;
  bool has_path_ = false;
  bool mission_complete_ = false;
  bool has_last_goal_ = false;
  ros::Time last_publish_time_;

  static bool isValidOrientation(const geometry_msgs::Quaternion& q) {
    const bool finite =
        std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
    const double norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return finite && norm2 > 1e-6;
  }

  static double pointDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static geometry_msgs::Point interpolatePoint(const geometry_msgs::Point& a,
                                               const geometry_msgs::Point& b,
                                               double ratio) {
    geometry_msgs::Point out;
    out.x = a.x + (b.x - a.x) * ratio;
    out.y = a.y + (b.y - a.y) * ratio;
    out.z = a.z + (b.z - a.z) * ratio;
    return out;
  }

  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    mission_path_ = *msg;
    cumulative_lengths_.clear();
    has_last_goal_ = false;
    mission_complete_ = false;

    if (mission_path_.poses.empty()) {
      has_path_ = false;
      ROS_WARN("[mission_manager] Received empty path, mission cleared.");
      return;
    }

    cumulative_lengths_.resize(mission_path_.poses.size(), 0.0);
    for (size_t i = 1; i < mission_path_.poses.size(); ++i) {
      cumulative_lengths_[i] =
          cumulative_lengths_[i - 1] +
          pointDistance(mission_path_.poses[i - 1].pose.position, mission_path_.poses[i].pose.position);
    }

    has_path_ = true;
    ROS_INFO("[mission_manager] Loaded mission path with %zu poses, total length %.2f m.",
             mission_path_.poses.size(),
             cumulative_lengths_.back());
  }

  double totalLength() const {
    return cumulative_lengths_.empty() ? 0.0 : cumulative_lengths_.back();
  }

  SampledPoint sampleAtProgress(double progress) const {
    SampledPoint sampled;
    if (mission_path_.poses.empty()) {
      return sampled;
    }

    if (mission_path_.poses.size() == 1) {
      sampled.position = mission_path_.poses.front().pose.position;
      sampled.orientation = mission_path_.poses.front().pose.orientation;
      return sampled;
    }

    const double clamped_progress = std::max(0.0, std::min(progress, totalLength()));
    sampled.progress = clamped_progress;

    if (clamped_progress >= totalLength()) {
      sampled.position = mission_path_.poses.back().pose.position;
      sampled.orientation = mission_path_.poses.back().pose.orientation;
      sampled.segment_index = mission_path_.poses.size() - 2;
      return sampled;
    }

    auto upper = std::upper_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), clamped_progress);
    size_t idx = 0;
    if (upper != cumulative_lengths_.begin()) {
      idx = static_cast<size_t>(std::distance(cumulative_lengths_.begin(), upper) - 1);
      idx = std::min(idx, mission_path_.poses.size() - 2);
    }

    const double segment_start = cumulative_lengths_[idx];
    const double segment_end = cumulative_lengths_[idx + 1];
    const double segment_length = std::max(segment_end - segment_start, 1e-6);
    const double ratio = (clamped_progress - segment_start) / segment_length;

    sampled.position = interpolatePoint(mission_path_.poses[idx].pose.position,
                                        mission_path_.poses[idx + 1].pose.position,
                                        ratio);
    sampled.orientation = ratio < 0.5 ? mission_path_.poses[idx].pose.orientation
                                      : mission_path_.poses[idx + 1].pose.orientation;
    sampled.segment_index = idx;
    return sampled;
  }

  SampledPoint closestPointOnPath(const geometry_msgs::Point& current_position) const {
    SampledPoint best_sample;
    double best_distance_sq = std::numeric_limits<double>::infinity();

    if (mission_path_.poses.empty()) {
      return best_sample;
    }

    if (mission_path_.poses.size() == 1) {
      best_sample.position = mission_path_.poses.front().pose.position;
      best_sample.orientation = mission_path_.poses.front().pose.orientation;
      return best_sample;
    }

    for (size_t i = 0; i + 1 < mission_path_.poses.size(); ++i) {
      const auto& a = mission_path_.poses[i].pose.position;
      const auto& b = mission_path_.poses[i + 1].pose.position;
      const double ab_x = b.x - a.x;
      const double ab_y = b.y - a.y;
      const double ab_z = b.z - a.z;
      const double ab_norm_sq = ab_x * ab_x + ab_y * ab_y + ab_z * ab_z;

      double ratio = 0.0;
      if (ab_norm_sq > 1e-9) {
        const double ap_x = current_position.x - a.x;
        const double ap_y = current_position.y - a.y;
        const double ap_z = current_position.z - a.z;
        ratio = (ap_x * ab_x + ap_y * ab_y + ap_z * ab_z) / ab_norm_sq;
        ratio = std::max(0.0, std::min(1.0, ratio));
      }

      const geometry_msgs::Point projection = interpolatePoint(a, b, ratio);
      const double dx = current_position.x - projection.x;
      const double dy = current_position.y - projection.y;
      const double dz = current_position.z - projection.z;
      const double distance_sq = dx * dx + dy * dy + dz * dz;
      const double progress = cumulative_lengths_[i] + std::sqrt(ab_norm_sq) * ratio;

      if (distance_sq < best_distance_sq - 1e-9 ||
          (std::abs(distance_sq - best_distance_sq) <= 1e-9 && progress > best_sample.progress)) {
        best_distance_sq = distance_sq;
        best_sample.position = projection;
        best_sample.orientation = ratio < 0.5 ? mission_path_.poses[i].pose.orientation
                                              : mission_path_.poses[i + 1].pose.orientation;
        best_sample.progress = progress;
        best_sample.segment_index = i;
      }
    }

    return best_sample;
  }

  bool tangentYaw(size_t segment_index, double* yaw) const {
    if (mission_path_.poses.size() < 2) {
      return false;
    }

    const size_t max_segment = mission_path_.poses.size() - 2;
    for (size_t offset = 0; offset <= max_segment; ++offset) {
      if (segment_index + offset <= max_segment) {
        const auto& a = mission_path_.poses[segment_index + offset].pose.position;
        const auto& b = mission_path_.poses[segment_index + offset + 1].pose.position;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        if (std::hypot(dx, dy) > 1e-6) {
          *yaw = std::atan2(dy, dx);
          return true;
        }
      }
      if (segment_index >= offset) {
        const auto& a = mission_path_.poses[segment_index - offset].pose.position;
        const auto& b = mission_path_.poses[segment_index - offset + 1].pose.position;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        if (std::hypot(dx, dy) > 1e-6) {
          *yaw = std::atan2(dy, dx);
          return true;
        }
      }
    }

    return false;
  }

  geometry_msgs::Quaternion orientationForTarget(const SampledPoint& target) const {
    if (use_final_waypoint_orientation_ &&
        target.progress >= totalLength() - 1e-3 &&
        isValidOrientation(mission_path_.poses.back().pose.orientation)) {
      return mission_path_.poses.back().pose.orientation;
    }

    double yaw = 0.0;
    if (tangentYaw(target.segment_index, &yaw)) {
      return tf::createQuaternionMsgFromYaw(yaw);
    }

    if (isValidOrientation(target.orientation)) {
      return target.orientation;
    }

    return tf::createQuaternionMsgFromYaw(0.0);
  }

  bool shouldPublishGoal(const geometry_msgs::PoseStamped& goal_msg) const {
    if (!has_last_goal_) {
      return true;
    }

    if (pointDistance(goal_msg.pose.position, last_goal_.pose.position) >= goal_update_distance_) {
      return true;
    }

    const double current_yaw = tf::getYaw(goal_msg.pose.orientation);
    const double last_yaw = tf::getYaw(last_goal_.pose.orientation);
    if (std::abs(normalizeAngle(current_yaw - last_yaw)) >= goal_update_yaw_) {
      return true;
    }

    return (goal_msg.header.stamp - last_publish_time_).toSec() >= republish_dt_;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!has_odom_ || !has_path_ || mission_complete_) {
      return;
    }

    const geometry_msgs::Point current_position = odom_.pose.pose.position;
    const SampledPoint closest = closestPointOnPath(current_position);
    const geometry_msgs::Point& final_point = mission_path_.poses.back().pose.position;

    if (pointDistance(current_position, final_point) <= goal_reached_distance_ &&
        closest.progress >= totalLength() - std::max(lookahead_distance_, 0.5)) {
      mission_complete_ = true;
      ROS_INFO("[mission_manager] Mission complete.");
      return;
    }

    const double target_progress = std::min(closest.progress + lookahead_distance_, totalLength());
    const SampledPoint target = sampleAtProgress(target_progress);

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id =
        mission_path_.header.frame_id.empty() ? default_frame_id_ : mission_path_.header.frame_id;
    goal_msg.pose.position = target.position;
    goal_msg.pose.orientation = orientationForTarget(target);

    if (!shouldPublishGoal(goal_msg)) {
      return;
    }

    goal_pub_.publish(goal_msg);
    last_goal_ = goal_msg;
    has_last_goal_ = true;
    last_publish_time_ = goal_msg.header.stamp;
  }
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_manager");
  MissionManager manager;
  ros::spin();
  return 0;
}
