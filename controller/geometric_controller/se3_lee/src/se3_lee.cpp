/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "se3_lee/se3_lee.h"
#include "se3_lee/jerk_tracking_control.h"
#include "se3_lee/nonlinear_attitude_control.h"
#include "se3_lee/nonlinear_geometric_control.h"

using namespace Eigen;
using namespace std;
// Constructor
Se3LeeCtrl::Se3LeeCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), dyn_config_server_(nh_private_), flightState_(WAITING_FOR_CONNECTED) {
  referenceSub_ =
      nh_.subscribe<geometry_msgs::TwistStamped>("reference/setpoint", 1, &Se3LeeCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe<std_msgs::Float32>("reference/yaw", 1, &Se3LeeCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1, &Se3LeeCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe<mavros_msgs::State>("mavros/state", 1, &Se3LeeCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &Se3LeeCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, &Se3LeeCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  // ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &Se3LeeCtrl::ctrltriggerCallback, this);
  
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &Se3LeeCtrl::cmdloopCallback,
                                   this);  // Define timer for constant loop rate

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  referencePoseEvalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/controller/reference_pose", 1);
  referenceVelEvalPub_ = nh_.advertise<geometry_msgs::TwistStamped>("/controller/reference_velocity", 1);
  referenceAccEvalPub_ = nh_.advertise<geometry_msgs::AccelStamped>("/controller/reference_accel", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  // posehistoryPub_ = nh_.advertise<nav_msgs::Path>("se3_lee/path", 10);
  // systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("/land", &Se3LeeCtrl::landCallback, this);

  // add
  flight_state_pub_ = nh_.advertise<std_msgs::Int8>("/flight_state", 1);

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("enable_auto_offboard", enable_auto_offboard_, sim_enable_);
  nh_private_.param<bool>("enable_auto_arm", enable_auto_arm_, sim_enable_);
  nh_private_.param<bool>("use_dynamic_reconfigure", use_dynamic_reconfigure_, false);
  nh_private_.param<bool>("auto_takeoff", auto_takeoff_, true);
  nh_private_.param<int>("offboard_warmup_count", offboard_warmup_count_, 80);
  nh_private_.param<double>("request_interval", request_interval_, 1.0);
  nh_private_.param<bool>("debug", debugFlag_, false);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 4.0);
  nh_private_.param<double>("max_vel", max_vel_, 2.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  
  double dx, dy, dz;
  nh_private_.param<double>("drag_dx", dx, 0.0);
  nh_private_.param<double>("drag_dy", dy, 0.0);
  nh_private_.param<double>("drag_dz", dz, 0.0);
  D_ << dx, dy, dz;

  double attctrl_tau;
  nh_private_.param<double>("attctrl_constant", attctrl_tau, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  // nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("takeoff_height", takeoff_height_, 2.0);
  nh_private_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
  nh_private_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
  nh_private_.param<double>("geo_fence/z", geo_fence_[2], 4.0);
  // nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  // nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  // nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);

  // targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetPos_.setZero();
  targetVel_.setZero();
  targetAcc_.setZero();
  targetJerk_.setZero();
  targetSnap_.setZero();
  mavPos_.setZero();
  mavVel_.setZero();
  
  prev_flightState_ = flightState_;
  if (offboard_warmup_count_ < 1) {
    offboard_warmup_count_ = 1;
  }
  if (request_interval_ < 0.1) {
    request_interval_ = 0.1;
  }
  last_mode_request_ = ros::Time(0);
  last_arm_request_ = ros::Time(0);
  // targetPos_[2] = takeoff_height_;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  bool jerk_enabled = false;
  if (!jerk_enabled) {
    if (ctrl_mode_ == ERROR_GEOMETRIC) {
      controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
    } else {
      controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    }
  } else {
    controller_ = std::make_shared<JerkTrackingControl>();
  }

  if (use_dynamic_reconfigure_) {
    dyn_config_callback_type_ = boost::bind(&Se3LeeCtrl::dynamicReconfigureCallback, this, _1, _2);
    dyn_config_server_.setCallback(dyn_config_callback_type_);
    ROS_INFO("se3_lee using dynamic_reconfigure for tuning parameters.");
  } else {
    ROS_INFO("se3_lee using static ROS parameters for tuning parameters.");
  }
}
Se3LeeCtrl::~Se3LeeCtrl() {
  // Destructor
}

void Se3LeeCtrl::targetCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) { // reference/setpoint
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg->twist.angular);
  targetVel_ = toEigen(msg->twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

void Se3LeeCtrl::yawtargetCallback(const std_msgs::Float32::ConstPtr &msg) { // reference/yaw
  if (!velocity_yaw_) mavYaw_ = double(msg->data); // false
}

void Se3LeeCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg) {
  // command/trajectory
  if (msg->points.empty()) {
    ROS_WARN("Received empty trajectory message");
    return;
  }
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg->points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_) { // false
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    mavYaw_ = rpy(2);
  }
}

void Se3LeeCtrl::mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // if (!received_home_pose) {
  //   received_home_pose = true;
  //   home_pose_ = msg->pose;
  //   ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  // }
  mavPos_ = toEigen(msg->pose.position);
  for(int i = 0;i<3;i++){
    if(mavPos_[i] > geo_fence_[i] || mavPos_[i] < -geo_fence_[i]){
      if(flightState_ != LANDING && flightState_ != LANDED)
        flightState_ = EMERGENCY;
      break;
    }
  }
  mavAtt_(0) = msg->pose.orientation.w;
  mavAtt_(1) = msg->pose.orientation.x;
  mavAtt_(2) = msg->pose.orientation.y;
  mavAtt_(3) = msg->pose.orientation.z;
}

void Se3LeeCtrl::mavtwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  mavVel_ = toEigen(msg->twist.linear);
  mavRate_ = toEigen(msg->twist.angular);
}

bool Se3LeeCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  ROS_INFO("trigger land!");
  flightState_ = LANDING;
  return true;
}

void Se3LeeCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  std_msgs::Int8 flight_state_msg;
  flight_state_msg.data = static_cast<int8_t>(flightState_);
  flight_state_pub_.publish(flight_state_msg);
  if (flightState_ != prev_flightState_) {
    ROS_WARN_STREAM("State changed from " << state2string(prev_flightState_) << " to " << state2string(flightState_));
    prev_flightState_ = flightState_;
  }
  switch (flightState_) {
    // case WAITING_FOR_HOME_POSE:
    //   waitForPredicate(&received_home_pose, "Waiting for home pose...");
    //   ROS_INFO("Got pose! Drone Ready to be armed.");
    //   ROS_INFO("Drone will takeoff to %lf[m]", takeoff_height_);
      // cout << takeoff_height_ <<std::endl;
      // if(!takeoffFlag_){
      //   double t = sqrt((2 * takeoff_height_)/max_fb_acc_);
      //   double takeoffVel = sqrt((2 * takeoff_height_ * max_fb_acc_));
      //   targetVel_[0] = mavVel_[0];
      //   targetVel_[1] = mavVel_[1];
      //   ROS_INFO("vel:%lf", takeoffVel);
      //   targetVel_[2] = min(takeoffVel, max_vel_);
      //   targetAcc_[2] = 0.5;
      // }
      // if (received_home_pose) {
      //   offboard_warmup_counter_ = 0;
      //   flightState_ = WAITING_FOR_OFFBOARD;
      // }
      
      // flightState_ = TAKEOFF;
      // break;
    case WAITING_FOR_CONNECTED: {
      ROS_INFO_ONCE("Waiting for FCU connection...");
      if (current_state_.connected) {
        ROS_INFO("Connected to FCU!");
        offboard_warmup_counter_ = 0;
        flightState_ = WAITING_FOR_OFFBOARD;
      }
      break;
    }
    case WAITING_FOR_OFFBOARD: {
      ROS_INFO_ONCE("Waiting for OFFBOARD mode and arming...");
      pubRateCommands(Eigen::Vector4d(0, 0, 0, 0.7), Eigen::Vector4d::Zero()); // send zero command to initialize the offboard mode
      ++offboard_warmup_counter_;
      const ros::Time now = ros::Time::now();
      TrySetOffboard(now);
      TryArm(now);

      if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
        if (auto_takeoff_) {
          targetPos_[2] = takeoff_height_;
          flightState_ = TAKEOFF;
        }else {
          flightState_ = MISSION_EXECUTION;
        }
      }
      break;
    }
    
    case TAKEOFF: {
      ROS_INFO_ONCE("Auto Taking off...");
      Eigen::Vector3d desired_acc;
      if(feedthrough_enable_) {
        desired_acc = targetAcc_;
      } else {
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
      }
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      pubRateCommands(cmdBodyRate_, q_des);
      if(fabs(mavPos_(2) - takeoff_height_) < 0.02){
        ROS_INFO("takeoff completed");
        flightState_ = MISSION_EXECUTION;
      }
      break;
    }

    case MISSION_EXECUTION: {
      ROS_INFO_ONCE("Mission execution...");
      Eigen::Vector3d desired_acc;
      if(feedthrough_enable_) {
        desired_acc = targetAcc_;
      } else {
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
      }
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      // pubReferencePose(targetPos_, q_des); // reference/pose
      pubRateCommands(cmdBodyRate_, q_des); // command/bodyrate_command
      // appendPoseHistory();
      // pubPoseHistory();
      break;
    }

    case LANDING: {
      landing_locked_ = true;
      mavros_msgs::SetMode land_set_mode;
      land_set_mode.request.custom_mode = "AUTO.LAND";
      if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
        flightState_ = LANDED;  
        ROS_INFO("land enabled");
      }
      // ros::spinOnce();
      break;
    }
    case LANDED:{
      if(!current_state_.armed){
        ROS_INFO("Landed. Please set to position control and disarm.");
        cmdloop_timer_.stop();
      }
      // ros::spinOnce();
      break;
    }
    case EMERGENCY:{
      ROS_WARN_STREAM_THROTTLE(2.0, "emergency! please switch to land");
      flightState_ = LANDING;
      // if(!current_state_.armed){
      //   ROS_WARN("Not arm, nothing to do");
      //   flightState_ = LANDED;
      //   break;
      // } 
      // geometry_msgs::PoseStamped msg;
      // msg.header.stamp = ros::Time::now();
      // msg.pose = home_pose_;
      // msg.pose.position.z = takeoff_height_;
      // target_pose_pub_.publish(msg);
      break;
    }
  }

  geometry_msgs::PoseStamped ref_eval_msg;
  ref_eval_msg.header.stamp = ros::Time::now();
  ref_eval_msg.header.frame_id = "map";
  ref_eval_msg.pose.position.x = targetPos_(0);
  ref_eval_msg.pose.position.y = targetPos_(1);
  ref_eval_msg.pose.position.z = targetPos_(2);
  ref_eval_msg.pose.orientation.w = 1.0;
  referencePoseEvalPub_.publish(ref_eval_msg);

  geometry_msgs::TwistStamped ref_vel_msg;
  ref_vel_msg.header = ref_eval_msg.header;
  ref_vel_msg.twist.linear.x = targetVel_(0);
  ref_vel_msg.twist.linear.y = targetVel_(1);
  ref_vel_msg.twist.linear.z = targetVel_(2);
  referenceVelEvalPub_.publish(ref_vel_msg);

  geometry_msgs::AccelStamped ref_acc_msg;
  ref_acc_msg.header = ref_eval_msg.header;
  ref_acc_msg.accel.linear.x = targetAcc_(0);
  ref_acc_msg.accel.linear.y = targetAcc_(1);
  ref_acc_msg.accel.linear.z = targetAcc_(2);
  referenceAccEvalPub_.publish(ref_acc_msg);
  // pubSystemStatus();
}

void Se3LeeCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) {
  current_state_ = *msg;
  if (current_state_.mode == "AUTO.LAND" && !landing_locked_) {
    landing_locked_ = true;
    ROS_WARN("se3_lee landing lock enabled (AUTO.LAND detected).");
  }
}

void Se3LeeCtrl::TrySetOffboard(const ros::Time &now) {
  if (landing_locked_ || !enable_auto_offboard_) {
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
    ROS_INFO_THROTTLE(2.0, "se3_lee requested OFFBOARD mode.");
  } else {
    ROS_WARN_THROTTLE(2.0, "se3_lee failed to request OFFBOARD mode.");
  }
  last_mode_request_ = now;
}

void Se3LeeCtrl::TryArm(const ros::Time &now) {
  if (landing_locked_ || !enable_auto_arm_) {
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

  arm_cmd_.request.value = true;
  if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
    ROS_INFO_THROTTLE(2.0, "se3_lee requested arming.");
  } else {
    ROS_WARN_THROTTLE(2.0, "se3_lee failed to arm.");
  }
  last_arm_request_ = now;
}

void Se3LeeCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void Se3LeeCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.orientation.w = target_attitude(0);
  msg.orientation.x = target_attitude(1);
  msg.orientation.y = target_attitude(2);
  msg.orientation.z = target_attitude(3);
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

// void Se3LeeCtrl::pubPoseHistory() {
//   nav_msgs::Path msg;

//   msg.header.stamp = ros::Time::now();
//   msg.header.frame_id = "map";
//   msg.poses = posehistory_vector_;

//   posehistoryPub_.publish(msg);
// }

// void Se3LeeCtrl::pubSystemStatus() {
//   mavros_msgs::CompanionProcessStatus msg;

//   msg.header.stamp = ros::Time::now();
//   msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
//   msg.state = (int)companion_state_;

//   systemstatusPub_.publish(msg);
// }

// void Se3LeeCtrl::appendPoseHistory() {
//   posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
//   if (posehistory_window_ != 0 && posehistory_vector_.size() > posehistory_window_) {
//     posehistory_vector_.pop_back();
//   }
// }

geometry_msgs::PoseStamped Se3LeeCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                  Eigen::Vector4d &orientation) {
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

Eigen::Vector3d Se3LeeCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) { //false
    mavYaw_ = getVelocityYaw(mavVel_);
  }

  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;

  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

  return a_des;
}

void Se3LeeCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
  // Reference attitude
  q_des = acc2quaternion(a_des, mavYaw_);

  controller_->Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
  bodyrate_cmd.head(3) = controller_->getDesiredRate();
  double thrust_command = controller_->getDesiredThrust().z();
  if(debugFlag_){
    ROS_INFO_STREAM_THROTTLE(5.0, "desired acc: " << a_des.transpose() << " thrust command: " << thrust_command);
  }
  bodyrate_cmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command +
                                      norm_thrust_offset_));  // Calculate thrustcontroller_->getDesiredThrust()(3);
}

Eigen::Vector3d Se3LeeCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}

Eigen::Vector4d Se3LeeCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

// bool Se3LeeCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
//   unsigned char mode = req.data;

//   ctrl_mode_ = mode;
//   res.success = ctrl_mode_;
//   res.message = "controller triggered";
//   return true;
// }

void Se3LeeCtrl::dynamicReconfigureCallback(se3_lee::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  }
  if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } 
  if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } 
  if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } 
  if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } 
  if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  } 
  if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
}
