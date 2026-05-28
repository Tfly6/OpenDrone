/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland
*/

#include <algorithm>
#include <cstdint>

#include <boost/bind.hpp>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_nonlinear_mpc/nonlinear_mpc_node.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>

namespace mav_control {

namespace {
constexpr double kTakeoffMidpointRatio = 0.5;
constexpr double kTakeoffReachTolerance = 0.15;
}  // namespace

NonLinearModelPredictiveControllerNode::NonLinearModelPredictiveControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      nonlinear_mpc_(nh, private_nh),
      controller_dyn_config_server_(private_nh),
      current_position_(Eigen::Vector3d::Zero()),
      has_odometry_(false),
      has_reference_(false),
      landing_locked_(false),
      takeoff_trajectory_sent_(false),
      offboard_warmup_counter_(0),
      mass_(1.0),
      hover_thrust_(0.5),
      thrust_min_(0.0),
      thrust_max_(1.0),
      yaw_rate_scale_(1.0),
      takeoff_height_(2.0),
      takeoff_target_z_(0.0),
      takeoff_duration_sec_(2.0),
      takeoff_trajectory_points_(3),
      control_rate_(100.0),
      odom_timeout_(0.5),
      enable_auto_offboard_(true),
      enable_auto_arm_(true),
      offboard_warmup_count_(80),
      request_interval_(1.0) {
  std::string command_pose_topic;
  std::string command_trajectory_topic;
  std::string odometry_topic;
  std::string command_output_topic;
  std::string attitude_target_topic;
  std::string state_topic;
  std::string set_mode_service;
  std::string arming_service;

  private_nh_.param<std::string>("command_pose_topic", command_pose_topic,
                                 mav_msgs::default_topics::COMMAND_POSE);
  private_nh_.param<std::string>("command_trajectory_topic", command_trajectory_topic,
                                 mav_msgs::default_topics::COMMAND_TRAJECTORY);
  private_nh_.param<std::string>("odometry_topic", odometry_topic,
                                 mav_msgs::default_topics::ODOMETRY);
  private_nh_.param<std::string>("command_output_topic", command_output_topic,
                                 mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST);
  private_nh_.param<std::string>("attitude_target_topic", attitude_target_topic,
                                 "/mavros/setpoint_raw/attitude");
  private_nh_.param<std::string>("state_topic", state_topic, "/mavros/state");
  private_nh_.param<std::string>("set_mode_service", set_mode_service, "/mavros/set_mode");
  private_nh_.param<std::string>("arming_service", arming_service, "/mavros/cmd/arming");

  private_nh_.param("mass", mass_, 1.0);
  private_nh_.param("hover_thrust", hover_thrust_, 0.5);
  private_nh_.param("thrust_min", thrust_min_, 0.0);
  private_nh_.param("thrust_max", thrust_max_, 1.0);
  private_nh_.param("yaw_rate_scale", yaw_rate_scale_, 1.0);
  private_nh_.param("control_rate", control_rate_, 100.0);
  private_nh_.param("odom_timeout", odom_timeout_, 0.5);
  private_nh_.param("enable_auto_offboard", enable_auto_offboard_, true);
  private_nh_.param("enable_auto_arm", enable_auto_arm_, true);
  private_nh_.param("auto_takeoff", auto_takeoff_, true);
  private_nh_.param("offboard_warmup_count", offboard_warmup_count_, 80);
  private_nh_.param("request_interval", request_interval_, 1.0);
  private_nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
  private_nh_.param<double>("takeoff_duration", takeoff_duration_sec_, 2.0);
  private_nh_.param<int>("takeoff_point_count", takeoff_trajectory_points_, 3);
  private_nh_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
  private_nh_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
  private_nh_.param<double>("geo_fence/z", geo_fence_[2], 4.0);

  takeoff_trajectory_points_ = std::max(2, std::min(3, takeoff_trajectory_points_));
  takeoff_duration_sec_ = std::max(0.5, takeoff_duration_sec_);

  dynamic_reconfigure::Server<mav_nonlinear_mpc::NonLinearMPCConfig>::CallbackType f_controller;
  f_controller = boost::bind(
      &NonLinearModelPredictiveControllerNode::ControllerDynConfigCallback,
      this, _1, _2);
  controller_dyn_config_server_.setCallback(f_controller);

  command_pose_subscriber_ = nh_.subscribe(command_pose_topic, 1,
                                           &NonLinearModelPredictiveControllerNode::CommandPoseCallback,
                                           this);
  command_trajectory_subscriber_ = nh_.subscribe(
      command_trajectory_topic, 1,
      &NonLinearModelPredictiveControllerNode::CommandTrajectoryCallback, this);
  odometry_subscriber_ = nh_.subscribe(odometry_topic, 1,
                                       &NonLinearModelPredictiveControllerNode::OdometryCallback,
                                       this, ros::TransportHints().tcpNoDelay());
  mavros_state_subscriber_ = nh_.subscribe(state_topic, 1,
                                           &NonLinearModelPredictiveControllerNode::MavrosStateCallback,
                                           this, ros::TransportHints().tcpNoDelay());

  command_publisher_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(command_output_topic, 1);
  attitude_target_publisher_ = nh_.advertise<mavros_msgs::AttitudeTarget>(attitude_target_topic, 1);

  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service);

  control_rate_ = std::max(20.0, control_rate_);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                   &NonLinearModelPredictiveControllerNode::ControlTimerCallback,
                                   this);

  flightState_ = WAITING_FOR_CONNECTED;
  prev_flightState_ = flightState_;

  last_mode_request_ = ros::Time(0);
  last_arm_request_ = ros::Time(0);
}

NonLinearModelPredictiveControllerNode::~NonLinearModelPredictiveControllerNode() {}

void NonLinearModelPredictiveControllerNode::ControllerDynConfigCallback(
    mav_nonlinear_mpc::NonLinearMPCConfig& config, uint32_t level) {
  (void)level;
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;

  Eigen::Vector3d r_command;
  Eigen::VectorXd control_limits(5);

  q_position << config.q_x, config.q_y, config.q_z;
  q_velocity << config.q_vx, config.q_vy, config.q_vz;
  q_attitude << config.q_roll, config.q_pitch;

  r_command << config.r_roll, config.r_pitch, config.r_thrust;

  control_limits << config.roll_max, config.pitch_max, config.yaw_rate_max,
      config.thrust_min, config.thrust_max;

  nonlinear_mpc_.setPositionPenality(q_position);
  nonlinear_mpc_.setVelocityPenality(q_velocity);
  nonlinear_mpc_.setAttitudePenality(q_attitude);
  nonlinear_mpc_.setCommandPenality(r_command);
  nonlinear_mpc_.setYawGain(config.K_yaw);
  nonlinear_mpc_.setControlLimits(control_limits);

  nonlinear_mpc_.setAltitudeIntratorGain(config.Ki_altitude);
  nonlinear_mpc_.setXYIntratorGain(config.Ki_xy);

  nonlinear_mpc_.setEnableIntegrator(config.enable_integrator);
  nonlinear_mpc_.setEnableOffsetFree(config.enable_offset_free);

  nonlinear_mpc_.applyParameters();
}

void NonLinearModelPredictiveControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  mav_msgs::EigenTrajectoryPoint reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);
  nonlinear_mpc_.setCommandTrajectoryPoint(reference);
  has_reference_ = true;
}

void NonLinearModelPredictiveControllerNode::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  if (msg->points.empty()) {
    return;
  }

  mav_msgs::EigenTrajectoryPointDeque reference_array;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &reference_array);
  nonlinear_mpc_.setCommandTrajectory(reference_array);
  has_reference_ = true;
}

void NonLinearModelPredictiveControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr& msg) {
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*msg, &odometry);
  odometry.timestamp_ns = ros::Time::now().toNSec();

  nonlinear_mpc_.setOdometry(odometry);
  current_position_ = odometry.position_W;
  current_yaw_ = tf::getYaw(msg->pose.pose.orientation);
  last_odometry_stamp_ = ros::Time::now();
  has_odometry_ = true;

  for (int i = 0; i < 3; ++i) {
    if (odometry.position_W[i] > geo_fence_[i] || odometry.position_W[i] < -geo_fence_[i]) {
      if (flightState_ != LANDING && flightState_ != LANDED) 
        flightState_ = EMERGENCY;
      break;
    }
  }
}

void NonLinearModelPredictiveControllerNode::MavrosStateCallback(
    const mavros_msgs::StateConstPtr& msg) {
  current_mavros_state_ = *msg;
  if (current_mavros_state_.mode == "AUTO.LAND" && !landing_locked_) {
    landing_locked_ = true;
    ROS_WARN("Nonlinear MPC node landing lock enabled (AUTO.LAND detected).");
  }
}

void NonLinearModelPredictiveControllerNode::TrySetOffboard(const ros::Time& now) {
  if (landing_locked_ || !enable_auto_offboard_) {
    return;
  }
  if (current_mavros_state_.mode == "OFFBOARD") {
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
    ROS_INFO_THROTTLE(2.0, "Nonlinear MPC node requested OFFBOARD mode.");
  } else {
    ROS_WARN_THROTTLE(2.0, "Nonlinear MPC node failed to request OFFBOARD mode.");
  }
  last_mode_request_ = now;
}

void NonLinearModelPredictiveControllerNode::TryArm(const ros::Time& now) {
  if (landing_locked_ || !enable_auto_arm_) {
    return;
  }
  if (current_mavros_state_.armed) {
    return;
  }
  if (enable_auto_offboard_ && current_mavros_state_.mode != "OFFBOARD") {
    return;
  }
  if ((now - last_arm_request_).toSec() < request_interval_) {
    return;
  }

  mavros_msgs::CommandBool arm_srv;
  arm_srv.request.value = true;
  if (arming_client_.call(arm_srv) && arm_srv.response.success) {
    ROS_INFO_THROTTLE(2.0, "Nonlinear MPC node requested arming.");
  } else {
    ROS_WARN_THROTTLE(2.0, "Nonlinear MPC node failed to arm.");
  }
  last_arm_request_ = now;
}

void NonLinearModelPredictiveControllerNode::PublishAttitudeTarget(
    const Eigen::Vector4d& rpy_thrust) {
  mavros_msgs::AttitudeTarget target;
  target.header.stamp = ros::Time::now();
  target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                     mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                     mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

  tf::Quaternion q;
  q.setRPY(rpy_thrust(0), rpy_thrust(1), rpy_thrust(2));
  q.normalize();
  target.orientation.x = q.x();
  target.orientation.y = q.y();
  target.orientation.z = q.z();
  target.orientation.w = q.w();

  const double normalized_thrust = hover_thrust_ * rpy_thrust(3) / 9.81;
  target.thrust = std::max(thrust_min_, std::min(thrust_max_, normalized_thrust));
  attitude_target_publisher_.publish(target);
}

void NonLinearModelPredictiveControllerNode::GenerateTakeoffTrajectory() {
  if (!has_odometry_) {
    ROS_WARN_THROTTLE(1.0, "Cannot generate takeoff trajectory without odometry.");
    return;
  }

  const double start_z = current_position_.z();
  const double climb_height = std::max(0.2, takeoff_height_);
  takeoff_target_z_ = start_z + climb_height;

  const int points = takeoff_trajectory_points_;
  const double dt = takeoff_duration_sec_ / static_cast<double>(points - 1);

  mav_msgs::EigenTrajectoryPointDeque takeoff_reference;
  for (int i = 0; i < points; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(points - 1);
    if (points == 3 && i == 1) {
      ratio = kTakeoffMidpointRatio;
    }

    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = current_position_;
    point.position_W.z() = start_z + climb_height * ratio;
    point.velocity_W = Eigen::Vector3d::Zero();
    point.acceleration_W = Eigen::Vector3d::Zero();
    point.setFromYaw(current_yaw_);
    point.setFromYawRate(0.0);
    point.time_from_start_ns = static_cast<int64_t>(1.0e9 * dt * static_cast<double>(i));
    takeoff_reference.push_back(point);
  }

  nonlinear_mpc_.setCommandTrajectory(takeoff_reference);
  has_reference_ = true;
  takeoff_trajectory_sent_ = true;

  ROS_INFO_STREAM("Generated " << points << "-point takeoff trajectory.");
}

bool NonLinearModelPredictiveControllerNode::IsTakeoffReached() const {
  if (!has_odometry_ || !takeoff_trajectory_sent_) {
    return false;
  }
  return current_position_.z() >= (takeoff_target_z_ - kTakeoffReachTolerance);
}

void NonLinearModelPredictiveControllerNode::ControlTimerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();

  if (!has_odometry_) {
    ROS_WARN_THROTTLE(1.0, "Nonlinear MPC node waiting for odometry.");
    return;
  }
  if ((now - last_odometry_stamp_).toSec() > odom_timeout_) {
    ROS_WARN_THROTTLE(1.0, "Nonlinear MPC node odometry timeout.");
    return;
  }

  if (flightState_ != prev_flightState_) {
    ROS_WARN_STREAM("State changed from " << state2string(prev_flightState_) << " to "
                    << state2string(flightState_));
    prev_flightState_ = flightState_;
  }

  switch (flightState_) {
    case WAITING_FOR_CONNECTED: {
      ROS_INFO_ONCE("Waiting for FCU connection...");
      if (current_mavros_state_.connected) {
        ROS_INFO("FCU Connected");
        offboard_warmup_counter_ = 0;
        flightState_ = WAITING_FOR_OFFBOARD;
      }
      break;
    }

    case WAITING_FOR_OFFBOARD: {
      ROS_INFO_ONCE("Waiting for OFFBOARD mode and arming...");
      PublishAttitudeTarget(Eigen::Vector4d(0.0, 0.0, 0.0, 9.81));
      ++offboard_warmup_counter_;
      TrySetOffboard(now);
      TryArm(now);

      if (current_mavros_state_.mode == "OFFBOARD" && current_mavros_state_.armed) {
        if(auto_takeoff_) {
          takeoff_trajectory_sent_ = false;
          flightState_ = TAKEOFF;
        } else {
          flightState_ = MISSION_EXECUTION;
        }
      }
      break;
    }

    case TAKEOFF: {
      if (!takeoff_trajectory_sent_) {
        GenerateTakeoffTrajectory();
      }
      ROS_INFO_ONCE("Auto Taking off...");
      Eigen::Vector4d rpyrate_thrust;
      nonlinear_mpc_.calculateRollPitchYawrateThrustCommand(&rpyrate_thrust);
      const Eigen::Vector4d rpy_thrust = nonlinear_mpc_.getCommandRollPitchYawThrust();
      PublishAttitudeTarget(rpy_thrust);

      if (IsTakeoffReached()) {
        ROS_INFO("Takeoff complete, entering mission execution.");
        flightState_ = MISSION_EXECUTION;
      }
      break;
    }

    case MISSION_EXECUTION: {
      if (!has_reference_) {
        ROS_WARN_THROTTLE(1.0, "Nonlinear MPC node waiting for reference.");
        break;
      }
      ROS_INFO_ONCE("Executing mission...");
      Eigen::Vector4d rpyrate_thrust;
      nonlinear_mpc_.calculateRollPitchYawrateThrustCommand(&rpyrate_thrust);
      const Eigen::Vector4d rpy_thrust = nonlinear_mpc_.getCommandRollPitchYawThrust();
      PublishAttitudeTarget(rpy_thrust);
      break;
    }

    case LANDING: {
      mavros_msgs::SetMode land_set_mode;
      land_set_mode.request.custom_mode = "AUTO.LAND";
      if (set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent) {
        ROS_INFO("AUTO.LAND enabled");
      }
      flightState_ = LANDED;
      break;
    }

    case LANDED: {
      if (!current_mavros_state_.armed) {
        ROS_INFO("Landed. Please set to position control and disarm.");
        control_timer_.stop();
      }
      break;
    }

    case EMERGENCY: {
      ROS_WARN_THROTTLE(1.0, "Emergency state entered, switching to LANDING.");
      flightState_ = LANDING;
      break;
    }

    default:
      break;
  }
}

}  // namespace mav_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "NonLinearModelPredictiveControllerNode");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  mav_control::NonLinearModelPredictiveControllerNode node(nh, private_nh);
  ros::spin();
  return 0;
}
