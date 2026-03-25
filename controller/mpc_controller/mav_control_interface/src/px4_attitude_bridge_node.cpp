#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace mav_control_interface {

class Px4AttitudeBridge
{
 public:
  Px4AttitudeBridge(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
      : nh_(nh),
        private_nh_(private_nh),
        has_odometry_(false),
        has_command_(false),
        current_yaw_(0.0),
          filter_initialized_(false),
        offboard_warmup_counter_(0),
        landing_locked_(false)
  {
    std::string command_topic;
    std::string odometry_topic;
    std::string attitude_target_topic;
    std::string state_topic;
    std::string set_mode_service;
    std::string arming_service;

    private_nh_.param<std::string>("command_topic", command_topic,
                                   mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST);
    private_nh_.param<std::string>("odometry_topic", odometry_topic, "/mavros/local_position/odom");
    private_nh_.param<std::string>("attitude_target_topic", attitude_target_topic,
                                   "/mavros/setpoint_raw/attitude");
    private_nh_.param<std::string>("state_topic", state_topic, "/mavros/state");
    private_nh_.param<std::string>("set_mode_service", set_mode_service, "/mavros/set_mode");
    private_nh_.param<std::string>("arming_service", arming_service, "/mavros/cmd/arming");

    private_nh_.param("mass", mass_, 1.0);
    private_nh_.param("hover_thrust", hover_thrust_, 0.5);
    private_nh_.param("odom_timeout", odom_timeout_, 0.5);
    private_nh_.param("command_timeout", command_timeout_, 0.5);
    private_nh_.param("thrust_min", thrust_min_, 0.0);
    private_nh_.param("thrust_max", thrust_max_, 1.0);
    private_nh_.param("yaw_rate_scale", yaw_rate_scale_, 1.0);
    private_nh_.param("setpoint_rate", setpoint_rate_, 50.0);
    private_nh_.param("enable_sim", enable_sim_, true);
    private_nh_.param("enable_command_lpf", enable_command_lpf_, true);
    private_nh_.param("attitude_lpf_tau", attitude_lpf_tau_, 0.08);
    private_nh_.param("yawrate_lpf_tau", yawrate_lpf_tau_, 0.12);
    private_nh_.param("thrust_lpf_tau", thrust_lpf_tau_, 0.15);
    private_nh_.param("enable_auto_offboard", enable_auto_offboard_, true);
    private_nh_.param("enable_auto_arm", enable_auto_arm_, true);
    private_nh_.param("offboard_warmup_count", offboard_warmup_count_, 80);
    private_nh_.param("request_interval", request_interval_, 1.0);

    if (setpoint_rate_ < 2.0) {
      ROS_WARN("setpoint_rate is too low for PX4 OFFBOARD, forcing to 20 Hz.");
      setpoint_rate_ = 20.0;
    }

    if (!enable_sim_) {
      enable_auto_offboard_ = false;
      enable_auto_arm_ = false;
      ROS_INFO("enable_sim is false: automatic OFFBOARD switch and arming are disabled.");
    }

    odometry_subscriber_ = nh_.subscribe(odometry_topic, 1,
                                         &Px4AttitudeBridge::OdometryCallback, this,
                                         ros::TransportHints().tcpNoDelay());
    command_subscriber_ = nh_.subscribe(command_topic, 1,
                                        &Px4AttitudeBridge::CommandCallback, this);
    state_subscriber_ = nh_.subscribe(state_topic, 1,
                                      &Px4AttitudeBridge::StateCallback, this,
                                      ros::TransportHints().tcpNoDelay());

    attitude_target_publisher_ = nh_.advertise<mavros_msgs::AttitudeTarget>(attitude_target_topic, 1);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / setpoint_rate_),
                                     &Px4AttitudeBridge::ControlTimerCallback, this);

    last_mode_request_ = ros::Time(0);
    last_arm_request_ = ros::Time(0);

    ROS_INFO_STREAM("px4_attitude_bridge started. command_topic=" << command_topic
                    << ", odometry_topic=" << odometry_topic
                    << ", attitude_target_topic=" << attitude_target_topic
                    << ", state_topic=" << state_topic
                    << ", enable_sim=" << (enable_sim_ ? "true" : "false")
                    << ", auto_offboard=" << (enable_auto_offboard_ ? "true" : "false")
                    << ", auto_arm=" << (enable_auto_arm_ ? "true" : "false"));
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher attitude_target_publisher_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;
  ros::Timer control_timer_;

  std::mutex mutex_;
  ros::Time last_odometry_stamp_;
  ros::Time last_command_stamp_;
  bool has_odometry_;
  bool has_command_;
  double current_yaw_;
  bool filter_initialized_;
  double filtered_roll_;
  double filtered_pitch_;
  double filtered_yaw_rate_;
  double filtered_thrust_;
  mav_msgs::RollPitchYawrateThrust last_command_;
  mavros_msgs::State current_state_;
  int offboard_warmup_counter_;
  bool landing_locked_;
  ros::Time last_mode_request_;
  ros::Time last_arm_request_;

  double mass_;
  double hover_thrust_;
  double odom_timeout_;
  double command_timeout_;
  double thrust_min_;
  double thrust_max_;
  double yaw_rate_scale_;
  double setpoint_rate_;
  bool enable_sim_;
  bool enable_command_lpf_;
  double attitude_lpf_tau_;
  double yawrate_lpf_tau_;
  double thrust_lpf_tau_;
  bool enable_auto_offboard_;
  bool enable_auto_arm_;
  int offboard_warmup_count_;
  double request_interval_;

  void StateCallback(const mavros_msgs::StateConstPtr& state_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_ = *state_msg;
    if (current_state_.mode == "AUTO.LAND" && !landing_locked_) {
      landing_locked_ = true;
      ROS_WARN("px4_attitude_bridge landing lock enabled (AUTO.LAND detected).");
    }
  }

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_yaw_ = tf::getYaw(odometry_msg->pose.pose.orientation);
    last_odometry_stamp_ = ros::Time::now();
    has_odometry_ = true;
  }

  void CommandCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& command_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = *command_msg;
    last_command_stamp_ = ros::Time::now();
    has_command_ = true;
  }

  mavros_msgs::AttitudeTarget BuildTargetFromState(const ros::Time& now)
  {
    mavros_msgs::AttitudeTarget target;
    target.header.stamp = now;
    target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                       mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;

    mav_msgs::RollPitchYawrateThrust command;
    bool command_valid = has_command_ && ((now - last_command_stamp_).toSec() <= command_timeout_);

    if (command_valid) {
      command = last_command_;
    } else {
      command.roll = 0.0;
      command.pitch = 0.0;
      command.yaw_rate = 0.0;
      command.thrust.z = mass_ * 9.81;
    }

    if (!has_odometry_) {
      ROS_WARN_THROTTLE(1.0, "px4_attitude_bridge waiting for odometry.");
      command.roll = 0.0;
      command.pitch = 0.0;
      command.yaw_rate = 0.0;
      command.thrust.z = mass_ * 9.81;
      filter_initialized_ = false;
    }

    if (has_odometry_ && (now - last_odometry_stamp_).toSec() > odom_timeout_) {
      ROS_WARN_THROTTLE(1.0, "px4_attitude_bridge odometry timeout, sending level hold.");
      command.roll = 0.0;
      command.pitch = 0.0;
      command.yaw_rate = 0.0;
      command.thrust.z = mass_ * 9.81;
      filter_initialized_ = false;
    }

    if (enable_command_lpf_) {
      const double dt = std::max(1.0 / std::max(setpoint_rate_, 1.0), 1.0e-3);
      const double tau_att = std::max(attitude_lpf_tau_, 1.0e-3);
      const double tau_yaw = std::max(yawrate_lpf_tau_, 1.0e-3);
      const double tau_thr = std::max(thrust_lpf_tau_, 1.0e-3);

      const double alpha_att = dt / (tau_att + dt);
      const double alpha_yaw = dt / (tau_yaw + dt);
      const double alpha_thr = dt / (tau_thr + dt);

      if (!filter_initialized_) {
        filtered_roll_ = command.roll;
        filtered_pitch_ = command.pitch;
        filtered_yaw_rate_ = command.yaw_rate;
        filtered_thrust_ = command.thrust.z;
        filter_initialized_ = true;
      } else {
        filtered_roll_ += alpha_att * (command.roll - filtered_roll_);
        filtered_pitch_ += alpha_att * (command.pitch - filtered_pitch_);
        filtered_yaw_rate_ += alpha_yaw * (command.yaw_rate - filtered_yaw_rate_);
        filtered_thrust_ += alpha_thr * (command.thrust.z - filtered_thrust_);
      }

      command.roll = filtered_roll_;
      command.pitch = filtered_pitch_;
      command.yaw_rate = filtered_yaw_rate_;
      command.thrust.z = filtered_thrust_;
    }

    tf::Quaternion q;
    q.setRPY(command.roll, command.pitch, current_yaw_);
    target.orientation.x = q.x();
    target.orientation.y = q.y();
    target.orientation.z = q.z();
    target.orientation.w = q.w();

    target.body_rate.z = command.yaw_rate * yaw_rate_scale_;

    const double gravity = 9.81;
    double normalized_thrust = 0.0;
    if (mass_ > 1.0e-6) {
      normalized_thrust = hover_thrust_ * command.thrust.z / (mass_ * gravity);
    }
    normalized_thrust = std::max(thrust_min_, std::min(thrust_max_, normalized_thrust));
    target.thrust = normalized_thrust;

    return target;
  }

  void TrySetOffboard(const ros::Time& now)
  {
    if (landing_locked_) {
      return;
    }
    if (!enable_auto_offboard_) {
      return;
    }
    if (current_state_.mode == "OFFBOARD") {
      return;
    }
    if (offboard_warmup_counter_ < offboard_warmup_count_) {
      return;
    }
    if ((now - last_mode_request_).toSec() < request_interval_) {
      return;
    }

    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = "OFFBOARD";
    if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
      ROS_INFO_THROTTLE(2.0, "px4_attitude_bridge requested OFFBOARD mode.");
    } else {
      ROS_WARN_THROTTLE(2.0, "px4_attitude_bridge failed to request OFFBOARD mode.");
    }
    last_mode_request_ = now;
  }

  void TryArm(const ros::Time& now)
  {
    if (landing_locked_) {
      return;
    }
    if (!enable_auto_arm_) {
      return;
    }
    if (current_state_.armed) {
      return;
    }
    if (enable_auto_offboard_ && current_state_.mode != "OFFBOARD") {
      return;
    }
    if ((now - last_arm_request_).toSec() < request_interval_) {
      return;
    }

    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;
    if (arming_client_.call(arm_srv) && arm_srv.response.success) {
      ROS_INFO_THROTTLE(2.0, "px4_attitude_bridge requested arming.");
    } else {
      ROS_WARN_THROTTLE(2.0, "px4_attitude_bridge failed to arm.");
    }
    last_arm_request_ = now;
  }

  void ControlTimerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mavros_msgs::AttitudeTarget target = BuildTargetFromState(now);
      attitude_target_publisher_.publish(target);

      if (current_state_.connected) {
        offboard_warmup_counter_++;
      } else {
        offboard_warmup_counter_ = 0;
        return;
      }

      TrySetOffboard(now);
      TryArm(now);
    }
  }
};

}  // namespace mav_control_interface

int main(int argc, char** argv)
{
  ros::init(argc, argv, "px4_attitude_bridge_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  mav_control_interface::Px4AttitudeBridge bridge(nh, private_nh);
  ros::spin();
  return 0;
}
