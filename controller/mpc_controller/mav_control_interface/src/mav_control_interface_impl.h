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

#ifndef MAV_CONTROL_INTERFACE_IMPL_H
#define MAV_CONTROL_INTERFACE_IMPL_H

#include <deque>
#include <string>

#include <ros/ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_control_interface/position_controller_interface.h>

#include "state_machine.h"

namespace mav_control_interface {

class MavControlInterfaceImpl
{
 public:
  MavControlInterfaceImpl(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                          std::shared_ptr<PositionControllerInterface> controller);

  virtual ~MavControlInterfaceImpl();

 private:
  static constexpr double kOdometryWatchdogTimeout = 1.0;  // seconds

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber command_trajectory_subscriber_;
  ros::Subscriber command_trajectory_array_subscriber_;
  ros::Timer odometry_watchdog_;

  ros::ServiceServer takeoff_server_;
  ros::ServiceServer land_server_;
  ros::ServiceServer back_to_position_hold_server_;
  ros::ServiceClient set_mode_client_;

  std::unique_ptr<state_machine::StateMachine> state_machine_;

  bool enable_auto_takeoff_;
  bool enable_auto_land_;
  bool enable_geofence_;
  bool auto_takeoff_require_offboard_;
  bool auto_takeoff_require_armed_;
  bool has_home_pose_;
  bool has_mavros_state_;
  bool auto_takeoff_triggered_;
  bool auto_land_triggered_;
  bool landing_locked_;
  mavros_msgs::State current_mavros_state_;
  Eigen::Vector3d home_position_;
  Eigen::Vector3d geofence_limit_;
  ros::Time last_mode_request_;
  double mode_request_interval_;
  std::string state_topic_;
  std::string set_mode_service_;

  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void CommandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void MavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
  void OdometryWatchdogCallback(const ros::TimerEvent& e);
  bool TakeoffCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);
  bool LandCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);
  bool BackToPositionHoldCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool IsAutoTakeoffReady() const;
  bool IsOutsideGeofence(const Eigen::Vector3d& position) const;
  void TriggerAutoLand(const std::string& reason);
  void publishAttitudeCommand(const mav_msgs::RollPitchYawrateThrust& command);
};

} /* namespace mav_control_interface */

#endif /* LOW_LEVEL_FLIGHT_MANAGER_H_ */
