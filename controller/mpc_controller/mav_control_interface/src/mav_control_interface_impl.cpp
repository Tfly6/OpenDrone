/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Geometry>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_control_interface_impl.h"
#include "parameters.h"

namespace mav_control_interface {

constexpr double MavControlInterfaceImpl::kOdometryWatchdogTimeout;

MavControlInterfaceImpl::MavControlInterfaceImpl(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                 std::shared_ptr<PositionControllerInterface> controller)
    : nh_(nh),
  private_nh_(private_nh),
  enable_auto_takeoff_(true),
  enable_auto_land_(true),
  enable_geofence_(true),
  auto_takeoff_require_offboard_(true),
  auto_takeoff_require_armed_(true),
  has_home_pose_(false),
  has_mavros_state_(false),
  auto_takeoff_triggered_(false),
  auto_land_triggered_(false),
  landing_locked_(false),
  home_position_(Eigen::Vector3d::Zero()),
  geofence_limit_(10.0, 10.0, 5.0),
  mode_request_interval_(1.0),
  state_topic_("/mavros/state"),
  set_mode_service_("/mavros/set_mode")
{
  ros::NodeHandle interface_nh(private_nh, "control_interface");

  odometry_watchdog_ = nh_.createTimer(ros::Duration(kOdometryWatchdogTimeout),
                                       &MavControlInterfaceImpl::OdometryWatchdogCallback, this, false, true);

  command_trajectory_subscriber_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1,
                                                 &MavControlInterfaceImpl::CommandPoseCallback, this);

  command_trajectory_array_subscriber_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &MavControlInterfaceImpl::CommandTrajectoryCallback, this);

  odometry_subscriber_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                       &MavControlInterfaceImpl::OdometryCallback, this,
                                       ros::TransportHints().tcpNoDelay());

  interface_nh.param<bool>("enable_auto_takeoff", enable_auto_takeoff_, true);
  interface_nh.param<bool>("enable_auto_land", enable_auto_land_, true);
  interface_nh.param<bool>("enable_geofence", enable_geofence_, true);
  interface_nh.param<bool>("auto_takeoff_require_offboard", auto_takeoff_require_offboard_, true);
  interface_nh.param<bool>("auto_takeoff_require_armed", auto_takeoff_require_armed_, true);
  interface_nh.param<double>("geo_fence/x", geofence_limit_.x(), 10.0);
  interface_nh.param<double>("geo_fence/y", geofence_limit_.y(), 10.0);
  interface_nh.param<double>("geo_fence/z", geofence_limit_.z(), 5.0);
  interface_nh.param<double>("mode_request_interval", mode_request_interval_, 1.0);
  interface_nh.param<std::string>("state_topic", state_topic_, "/mavros/state");
  interface_nh.param<std::string>("set_mode_service", set_mode_service_, "/mavros/set_mode");

  mavros_state_subscriber_ = nh_.subscribe(state_topic_, 1,
                                           &MavControlInterfaceImpl::MavrosStateCallback,
                                           this, ros::TransportHints().tcpNoDelay());
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service_);

  takeoff_server_ = nh.advertiseService("takeoff", &MavControlInterfaceImpl::TakeoffCallback, this);
  land_server_ = nh.advertiseService("land", &MavControlInterfaceImpl::LandCallback, this);
  back_to_position_hold_server_ = nh.advertiseService("back_to_position_hold",
                                                      &MavControlInterfaceImpl::BackToPositionHoldCallback,
                                                      this);

  state_machine_.reset(new state_machine::StateMachine(nh_, private_nh_, controller));

  Parameters p;
  interface_nh.param("takeoff_distance", p.takeoff_distance_, Parameters::kDefaultTakeoffDistance);
  interface_nh.param("takeoff_time", p.takeoff_time_, Parameters::kDefaultTakeoffTime);
  state_machine_->SetParameters(p);

  ROS_INFO_STREAM("Created control interface for controller " << controller->getName());
  ROS_INFO_STREAM("Control safety config: auto_takeoff=" << (enable_auto_takeoff_ ? "true" : "false")
                  << ", require_offboard_for_takeoff=" << (auto_takeoff_require_offboard_ ? "true" : "false")
                  << ", require_armed_for_takeoff=" << (auto_takeoff_require_armed_ ? "true" : "false")
                  << ", auto_land=" << (enable_auto_land_ ? "true" : "false")
                  << ", geofence=" << (enable_geofence_ ? "true" : "false")
                  << ", geofence_limit=[" << geofence_limit_.x() << ", "
                  << geofence_limit_.y() << ", " << geofence_limit_.z() << "]");

  state_machine_->start();
}

MavControlInterfaceImpl::~MavControlInterfaceImpl()
{
  state_machine_->stop();
}

void MavControlInterfaceImpl::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

  mav_msgs::EigenTrajectoryPoint reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);

  mav_msgs::EigenTrajectoryPointDeque references;
  references.push_back(reference);

  state_machine_->process_event(state_machine::ReferenceUpdate(references));
}

void MavControlInterfaceImpl::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  int array_size = msg->points.size();
  if (array_size == 0)
    return;

  mav_msgs::EigenTrajectoryPointDeque references;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);

  state_machine_->process_event(state_machine::ReferenceUpdate(references));
}

void MavControlInterfaceImpl::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("Control interface got first odometry message.");
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
  // Stamp odometry upon reception to be robust against timestamps "in the future".
  odometry.timestamp_ns = ros::Time::now().toNSec();
  state_machine_->process_event(state_machine::OdometryUpdate(odometry));

  if (!has_home_pose_) {
    home_position_ = odometry.position_W;
    has_home_pose_ = true;
    ROS_INFO_STREAM("Home position initialized to ["
                    << home_position_.x() << ", "
                    << home_position_.y() << ", "
                    << home_position_.z() << "]");
  }

  if (enable_auto_takeoff_ && !auto_takeoff_triggered_ && !landing_locked_) {
    if (IsAutoTakeoffReady()) {
      state_machine_->process_event(state_machine::Takeoff());
      auto_takeoff_triggered_ = true;
      ROS_INFO("Auto takeoff triggered.");
    } else {
      ROS_INFO_THROTTLE(1.0,
                        "Auto takeoff waiting: mavros_state=%s mode=%s armed=%s (require_offboard=%s require_armed=%s)",
                        has_mavros_state_ ? "yes" : "no",
                        has_mavros_state_ ? current_mavros_state_.mode.c_str() : "N/A",
                        (has_mavros_state_ && current_mavros_state_.armed) ? "true" : "false",
                        auto_takeoff_require_offboard_ ? "true" : "false",
                        auto_takeoff_require_armed_ ? "true" : "false");
    }
  }

  if (enable_geofence_ && enable_auto_land_ && has_home_pose_ && !auto_land_triggered_) {
    if (IsOutsideGeofence(odometry.position_W)) {
      TriggerAutoLand("Geofence exceeded.");
    }
  }
}

void MavControlInterfaceImpl::MavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  has_mavros_state_ = true;
  current_mavros_state_ = *msg;
  if (msg->mode == "AUTO.LAND" && !landing_locked_) {
    landing_locked_ = true;
    auto_land_triggered_ = true;
    ROS_WARN("Landing lock enabled (AUTO.LAND detected).");
  }
}

bool MavControlInterfaceImpl::IsAutoTakeoffReady() const
{
  if (!has_mavros_state_) {
    return false;
  }
  if (auto_takeoff_require_offboard_ && current_mavros_state_.mode != "OFFBOARD") {
    return false;
  }
  if (auto_takeoff_require_armed_ && !current_mavros_state_.armed) {
    return false;
  }
  return true;
}

void MavControlInterfaceImpl::OdometryWatchdogCallback(const ros::TimerEvent& e)
{
  state_machine_->process_event(state_machine::OdometryWatchdog());
}

bool MavControlInterfaceImpl::TakeoffCallback(std_srvs::Empty::Request& request,
                                              std_srvs::Empty::Response& response)
{
  if (landing_locked_) {
    ROS_WARN("Takeoff rejected: landing lock is active.");
    return true;
  }

  ROS_INFO("Take off event sent");
  state_machine_->process_event(state_machine::Takeoff());
  auto_takeoff_triggered_ = true;
  return true;
}

bool MavControlInterfaceImpl::LandCallback(std_srvs::Empty::Request& request,
                                           std_srvs::Empty::Response& response)
{
  TriggerAutoLand("Land service requested.");
  return true;
}

bool MavControlInterfaceImpl::BackToPositionHoldCallback(std_srvs::Empty::Request& request,
                                                         std_srvs::Empty::Response& response)
{
  if (landing_locked_) {
    ROS_WARN("Back-to-position-hold rejected: landing lock is active.");
    return true;
  }

  state_machine_->process_event(state_machine::BackToPositionHold());
  return true;
}

bool MavControlInterfaceImpl::IsOutsideGeofence(const Eigen::Vector3d& position) const
{
  const Eigen::Vector3d delta = (position - home_position_).cwiseAbs();
  return delta.x() > geofence_limit_.x() ||
      delta.y() > geofence_limit_.y() ||
      delta.z() > geofence_limit_.z();
}

void MavControlInterfaceImpl::TriggerAutoLand(const std::string& reason)
{
  if (auto_land_triggered_) {
    return;
  }

  const ros::Time now = ros::Time::now();
  if ((now - last_mode_request_).toSec() < mode_request_interval_) {
    return;
  }

  mavros_msgs::SetMode land_mode;
  land_mode.request.custom_mode = "AUTO.LAND";

  if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
    auto_land_triggered_ = true;
    landing_locked_ = true;
    ROS_WARN_STREAM(reason << " Switching to AUTO.LAND.");
  } else {
    ROS_WARN_STREAM("Failed to request AUTO.LAND. " << reason);
  }

  last_mode_request_ = now;
}

}  // end namespace mav_control_interface

