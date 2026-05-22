/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 */

#ifndef LINEAR_MPC_NODE_H
#define LINEAR_MPC_NODE_H

#include <string>

#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_linear_mpc/LinearMPCConfig.h>
#include <mav_linear_mpc/linear_mpc.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace mav_control {

class LinearModelPredictiveControllerNode {
 public:
  LinearModelPredictiveControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LinearModelPredictiveControllerNode();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void DynConfigCallback(mav_linear_mpc::LinearMPCConfig& config, uint32_t level);

  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void CommandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void MavrosStateCallback(const mavros_msgs::StateConstPtr& msg);
  void ControlTimerCallback(const ros::TimerEvent&);

  void TrySetOffboard(const ros::Time& now);
  void TryArm(const ros::Time& now);
  void PublishAttitudeTarget(const Eigen::Vector4d& rpy_thrust);
  void GenerateTakeoffTrajectory();
  bool IsTakeoffReached() const;

  enum FlightState {
    WAITING_FOR_CONNECTED,
    WAITING_FOR_OFFBOARD,
    TAKEOFF,
    MISSION_EXECUTION,
    LANDING,
    LANDED,
    EMERGENCY,
  } flightState_, prev_flightState_;

  std::string state2string(FlightState state) {
    switch (state) {
      case WAITING_FOR_CONNECTED:
        return "WAITING_FOR_CONNECTED";
      case WAITING_FOR_OFFBOARD:
        return "WAITING_FOR_OFFBOARD";
      case MISSION_EXECUTION:
        return "MISSION_EXECUTION";
      case LANDING:
        return "LANDING";
      case LANDED:
        return "LANDED";
      case TAKEOFF:
        return "TAKEOFF";
      case EMERGENCY:
        return "EMERGENCY";
      default:
        return "UNKNOWN_STATE";
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  LinearModelPredictiveController linear_mpc_;
  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig> dyn_config_server_;

  ros::Subscriber command_pose_subscriber_;
  ros::Subscriber command_trajectory_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber mavros_state_subscriber_;

  ros::Publisher command_publisher_;
  ros::Publisher attitude_target_publisher_;

  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;

  ros::Timer control_timer_;

  mavros_msgs::State current_mavros_state_;
  Eigen::Vector3d geo_fence_;
  Eigen::Vector3d current_position_;

  bool has_odometry_;
  bool has_reference_;
  bool landing_locked_;
  bool takeoff_trajectory_sent_;
  int offboard_warmup_counter_;

  ros::Time last_odometry_stamp_;
  ros::Time last_mode_request_;
  ros::Time last_arm_request_;

  double current_yaw_;

  double mass_;
  double hover_thrust_;
  double thrust_min_;
  double thrust_max_;
  double yaw_rate_scale_;
  double takeoff_height_;
  double takeoff_target_z_;
  double takeoff_duration_sec_;
  int takeoff_trajectory_points_;

  double control_rate_;
  double odom_timeout_;
  bool enable_auto_offboard_;
  bool enable_auto_arm_;
  int offboard_warmup_count_;
  double request_interval_;
};

}  // namespace mav_control

#endif
