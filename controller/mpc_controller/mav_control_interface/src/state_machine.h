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

/**
 * This implements the state machine that can be seen and edited here:
 * https://drive.google.com/file/d/0B6zP-RNkbXDcRFZENUF6MEhFbW8/view?usp=sharing
 * The pdf and svg can be found in the resource folder.
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <algorithm>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <mav_control_interface/position_controller_interface.h>
#include "parameters.h"

#include <visualization_msgs/Marker.h>

namespace mav_control_interface {

namespace state_machine {

namespace msm_front = boost::msm::front;
namespace mpl = boost::mpl;

struct ReferenceUpdate
{
  ReferenceUpdate(const mav_msgs::EigenTrajectoryPointDeque& _references)
      : references(_references)
  {
  }

  mav_msgs::EigenTrajectoryPointDeque references;
};

struct OdometryUpdate
{
  OdometryUpdate(const mav_msgs::EigenOdometry& _odometry)
      : odometry(_odometry)
  {
  }

  mav_msgs::EigenOdometry odometry;
};

struct BackToPositionHold {};
struct Takeoff {};
struct OdometryWatchdog {};

class StateMachineDefinition;
typedef boost::msm::back::state_machine<StateMachineDefinition, boost::msm::back::mpl_graph_fsm_check> StateMachine;

class StateMachineDefinition : public msm_front::state_machine_def<StateMachineDefinition>
{
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  struct Inactive;
  struct HaveOdometry;
  struct PositionHold;

  struct SetReferencePosition;
  struct SetOdometry;
  struct ComputeCommand;
  struct SetTakeoffCommands;
  struct PrintOdometryWatchdogWarning;

  struct OdometryOutdated;

 public:
  typedef Inactive initial_state;
  typedef int no_exception_thrown;

  typedef msm_front::ActionSequence_<mpl::vector<SetOdometry, ComputeCommand> > SetOdometryAndCompute;
  typedef msm_front::none InternalTransition;
  typedef msm_front::none NoAction;
  typedef msm_front::none NoGuard;

  struct transition_table : boost::mpl::vector<
      msm_front::Row<Inactive, ReferenceUpdate, PositionHold, SetReferencePosition, NoGuard >,
      msm_front::Row<Inactive, OdometryWatchdog, InternalTransition, PrintOdometryWatchdogWarning, OdometryOutdated >,
      msm_front::Row<Inactive, OdometryUpdate, HaveOdometry, SetOdometry, NoGuard >,
      msm_front::Row<Inactive, Takeoff, PositionHold, SetTakeoffCommands, NoGuard>,

      msm_front::Row<HaveOdometry, OdometryUpdate, InternalTransition, SetOdometry, NoGuard >,
      msm_front::Row<HaveOdometry, ReferenceUpdate, PositionHold, SetReferencePosition, NoGuard >,
      msm_front::Row<HaveOdometry, Takeoff, PositionHold, SetTakeoffCommands, NoGuard >,
      msm_front::Row<HaveOdometry, OdometryWatchdog, Inactive, PrintOdometryWatchdogWarning, OdometryOutdated >,

      msm_front::Row<PositionHold, BackToPositionHold, InternalTransition, NoAction, NoGuard>,
      msm_front::Row<PositionHold, Takeoff, InternalTransition, SetTakeoffCommands, NoGuard>,
      msm_front::Row<PositionHold, OdometryUpdate, InternalTransition, SetOdometryAndCompute, NoGuard>,
      msm_front::Row<PositionHold, ReferenceUpdate, InternalTransition, SetReferencePosition, NoGuard >,
      msm_front::Row<PositionHold, OdometryWatchdog, Inactive, PrintOdometryWatchdogWarning, OdometryOutdated >
      >
  {
  };

 public:
  StateMachineDefinition(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh,
                         std::shared_ptr<PositionControllerInterface> controller);

  template<class Event, class FSM>
  void on_entry(Event const&, FSM& fsm)
  {
    fsm.PublishStateInfo("entering StateMachine");
  }

  template<class Event, class FSM>
  void on_exit(Event const&, FSM& fsm)
  {
    fsm.PublishStateInfo("leaving StateMachine");
  }

  void SetParameters(const Parameters& parameters);

 private:
  static constexpr int64_t kOdometryOutdated_ns = 1000000000;
  std::string reference_frame_id_;
  std::shared_ptr<PositionControllerInterface> controller_;
  ros::Publisher command_publisher_;
  ros::Publisher state_info_publisher_;

  tf::TransformBroadcaster transform_broadcaster_;
  ros::Publisher current_reference_publisher_;
  ros::Publisher predicted_state_publisher_;
  ros::Publisher full_predicted_state_publisher_;
  Parameters parameters_;
  mav_msgs::EigenOdometry current_state_;
  mav_msgs::EigenTrajectoryPointDeque current_reference_queue_;

  void PublishAttitudeCommand(const mav_msgs::EigenRollPitchYawrateThrust& command) const;
  void PublishStateInfo(const std::string& info);
  void PublishCurrentReference();
  void PublishPredictedState();

  struct Inactive : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      fsm.PublishStateInfo("Inactive");
    }
  };

  struct HaveOdometry : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      fsm.PublishStateInfo("HaveOdometry");
    }
  };

  struct PositionHold : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      fsm.PublishStateInfo("PositionHold");
    }
  };

  struct SetReferencePosition
  {
    template<class FSM, class SourceState, class TargetState>
    void operator()(const ReferenceUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      if(evt.references.size() == 1){
        fsm.controller_->setReference(evt.references.at(0));
      }else{
        fsm.controller_->setReferenceArray(evt.references);
      }

      fsm.current_reference_queue_ = evt.references;
    }
  };

  struct SetOdometry
  {
    template<class FSM, class SourceState, class TargetState>
    void operator()(const OdometryUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      fsm.current_state_ = evt.odometry;
      fsm.controller_->setOdometry(evt.odometry);
    }
  };

  struct ComputeCommand
  {
    template<class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& evt, FSM& fsm, SourceState&, TargetState&)
    {
      mav_msgs::EigenRollPitchYawrateThrust command;
      fsm.controller_->calculateRollPitchYawrateThrustCommand(&command);
      fsm.PublishAttitudeCommand(command);
      fsm.PublishCurrentReference();
      fsm.PublishPredictedState();
    }
  };

  struct SetTakeoffCommands
  {
    template<class FSM, class SourceState, class TargetState>
    void operator()(const Takeoff& evt, FSM& fsm, SourceState& src_state, TargetState&)
    {
      const Parameters& p = fsm.parameters_;
      mav_msgs::EigenOdometry& current_state = fsm.current_state_;
      mav_msgs::EigenTrajectoryPointDeque& current_reference_queue = fsm.current_reference_queue_;
      current_reference_queue.clear();

      const double takeoff_time = std::max(0.1, p.takeoff_time_);
      const int64_t takeoff_time_ns = static_cast<int64_t>(takeoff_time * 1.0e9);
      const double current_yaw = mav_msgs::yawFromQuaternion(current_state.orientation_W_B);

      mav_msgs::EigenTrajectoryPoint start_point;
      start_point.time_from_start_ns = 0;
      start_point.position_W = current_state.position_W;
      start_point.velocity_W = Eigen::Vector3d::Zero();
      start_point.acceleration_W = Eigen::Vector3d::Zero();
      start_point.setFromYaw(current_yaw);
      start_point.setFromYawRate(0.0);
      current_reference_queue.push_back(start_point);

      mav_msgs::EigenTrajectoryPoint trajectory_point;
      trajectory_point.time_from_start_ns = takeoff_time_ns;
      trajectory_point.position_W = current_state.position_W;
      trajectory_point.position_W.z() += p.takeoff_distance_;
      trajectory_point.velocity_W = Eigen::Vector3d::Zero();
      trajectory_point.acceleration_W = Eigen::Vector3d::Zero();
      trajectory_point.setFromYaw(current_yaw);
      trajectory_point.setFromYawRate(0.0);
      current_reference_queue.push_back(trajectory_point);

      ROS_INFO_STREAM("Takeoff trajectory command: start=" << start_point.position_W.transpose()
                      << ", end=" << trajectory_point.position_W.transpose()
                      << ", duration=" << takeoff_time << " s");
      fsm.controller_->setReferenceArray(current_reference_queue);
    }
  };

  struct PrintOdometryWatchdogWarning
  {
    template<class FSM, class Evt, class SourceState, class TargetState>
    void operator()(const Evt& evt, FSM& fsm, SourceState&, TargetState&)
    {
      ROS_WARN_STREAM("No odometry message received in the last "
                      << kOdometryOutdated_ns / 1000000000.0 << " seconds!");
    }
  };

  struct OdometryOutdated
  {
    template<class FSM, class SourceState, class TargetState>
    bool operator()(const OdometryWatchdog& evt, FSM& fsm, SourceState&, TargetState&)
    {
      return std::abs(static_cast<int64_t>(ros::Time::now().toNSec()) - fsm.current_state_.timestamp_ns) > kOdometryOutdated_ns;
    }
  };

};

} // end namespace state_machine

} // namespace mav_control_interface

#endif /* STATE_MACHINE_H_ */
