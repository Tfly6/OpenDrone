/*
 * Copyright (c) 2017, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
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

#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>

#include <cmath>

TrajectorySamplerNode::TrajectorySamplerNode(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_(0.0),
      loop_trajectory_(false),
      trigger_sent_(false) {
  nh_private_.param("publish_whole_trajectory", publish_whole_trajectory_,
                    publish_whole_trajectory_);
  nh_private_.param("dt", dt_, dt_);
  nh_private_.param("loop_trajectory", loop_trajectory_, loop_trajectory_);
  nh_private_.param<std::string>("planner_output_topic", planner_output_topic_,
                                 "/planner/output");
  nh_private_.param<std::string>("frame_id", frame_id_, "map");

  planner_output_pub_ = nh_.advertise<opendrone::PlannerOutput>(
      planner_output_topic_, 1, true);
  traj_trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "traj_start_trigger", 1);
  trajectory_sub_ = nh_.subscribe(
      "path_segments", 10, &TrajectorySamplerNode::pathSegmentsCallback, this);
  trajectory4D_sub_ = nh_.subscribe(
      "path_segments_4D", 10, &TrajectorySamplerNode::pathSegments4DCallback, this);
  stop_srv_ = nh_.advertiseService(
      "stop_sampling", &TrajectorySamplerNode::stopSamplingCallback, this);
  position_hold_client_ =
      nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

  const bool oneshot = false;
  const bool autostart = false;
  publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                   &TrajectorySamplerNode::commandTimerCallback,
                                   this, oneshot, autostart);
}

TrajectorySamplerNode::~TrajectorySamplerNode() { publish_timer_.stop(); }

void TrajectorySamplerNode::pathSegmentsCallback(
    const mav_planning_msgs::PolynomialTrajectory& segments_message) {
  if (segments_message.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu segments",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::pathSegments4DCallback(
    const mav_planning_msgs::PolynomialTrajectory4D& segments_message) {
  if (segments_message.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu segments",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::processTrajectory() {
  trigger_sent_ = false;
  ++trajectory_id_;
  start_time_ = ros::Time::now();
  // Call the service call to takeover publishing commands.
  if (position_hold_client_.exists()) {
    std_srvs::Empty empty_call;
    position_hold_client_.call(empty_call);
  }

  if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
    ROS_INFO("Publishing the whole trajectory.");
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
                                                     &trajectory_points);
    // trajectory_msgs::MultiDOFJointTrajectory msg_pub;
    // mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
    // command_pub_.publish(msg_pub);
    planner_output_pub_.publish(buildPlannerOutput(trajectory_points, false));
  } else {
    publish_timer_.start();
    current_sample_time_ = 0.0;
  }
}

bool TrajectorySamplerNode::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_.stop();
  return true;
}

void TrajectorySamplerNode::commandTimerCallback(const ros::TimerEvent&) {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    // trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      ROS_WARN("Trajectory sampler: failed to sample trajectory at time %f",
               current_sample_time_);
      publish_timer_.stop();
    }
    // mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    // msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    // command_pub_.publish(msg);
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    trajectory_points.push_back(trajectory_point);
    planner_output_pub_.publish(buildPlannerOutput(trajectory_points, true));
    current_sample_time_ += dt_;
  } else {
    publish_timer_.stop();
    if (loop_trajectory_ && !trigger_sent_) {
      trigger_sent_ = true;
      geometry_msgs::PoseStamped trigger_msg;
      trigger_msg.header.stamp = ros::Time::now();
      trigger_msg.header.frame_id = "map";
      traj_trigger_pub_.publish(trigger_msg);
      ROS_INFO("Trajectory sampler: loop trigger sent.");
    }
  }
}

opendrone::PlannerOutput TrajectorySamplerNode::buildPlannerOutput(
    const mav_msgs::EigenTrajectoryPoint::Vector& trajectory_points,
    bool split_samples) const {
  opendrone::PlannerOutput msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.trajectory_id = trajectory_id_;
  msg.is_horizon = false;
  msg.trajectory_start_time = start_time_;

  if (!trajectory_points.empty()) {
    msg.points.reserve(trajectory_points.size());
    for (const auto& trajectory_point : trajectory_points) {
      msg.points.push_back(toPlannerOutputPoint(trajectory_point));
    }
  }
  return msg;
}

opendrone::PlannerOutputPoint TrajectorySamplerNode::toPlannerOutputPoint(
    const mav_msgs::EigenTrajectoryPoint& trajectory_point) const {
  opendrone::PlannerOutputPoint pt;
  ros::Duration time_from_start;
  time_from_start.fromNSec(
      static_cast<int64_t>(std::max<int64_t>(trajectory_point.time_from_start_ns, 0)));
  pt.time_from_start = time_from_start;
  pt.valid_mask =
      opendrone::PlannerOutputPoint::VALID_POSITION |
      opendrone::PlannerOutputPoint::VALID_VELOCITY |
      opendrone::PlannerOutputPoint::VALID_ACCELERATION |
      opendrone::PlannerOutputPoint::VALID_JERK |
      opendrone::PlannerOutputPoint::VALID_SNAP |
      opendrone::PlannerOutputPoint::VALID_YAW |
      opendrone::PlannerOutputPoint::VALID_YAW_RATE;

  pt.position.x = trajectory_point.position_W.x();
  pt.position.y = trajectory_point.position_W.y();
  pt.position.z = trajectory_point.position_W.z();

  pt.velocity.x = trajectory_point.velocity_W.x();
  pt.velocity.y = trajectory_point.velocity_W.y();
  pt.velocity.z = trajectory_point.velocity_W.z();

  pt.acceleration.x = trajectory_point.acceleration_W.x();
  pt.acceleration.y = trajectory_point.acceleration_W.y();
  pt.acceleration.z = trajectory_point.acceleration_W.z();

  pt.jerk.x = trajectory_point.jerk_W.x();
  pt.jerk.y = trajectory_point.jerk_W.y();
  pt.jerk.z = trajectory_point.jerk_W.z();

  pt.snap.x = trajectory_point.snap_W.x();
  pt.snap.y = trajectory_point.snap_W.y();
  pt.snap.z = trajectory_point.snap_W.z();

  pt.yaw = trajectory_point.getYaw();
  pt.yaw_rate = trajectory_point.getYawRate();
  return pt;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_sampler_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  TrajectorySamplerNode trajectory_sampler_node(nh, nh_private);
  ROS_INFO("Initialized trajectory sampler.");
  ros::spin();
}
