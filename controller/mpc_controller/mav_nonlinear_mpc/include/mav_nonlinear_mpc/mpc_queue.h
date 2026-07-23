/*
 * Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland
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

#ifndef MAV_NONLINEAR_MPC_MPC_QUEUE_H_
#define MAV_NONLINEAR_MPC_MPC_QUEUE_H_

#include <deque>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

namespace mav_control {

typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;

class MPCQueue {
 public:
  MPCQueue(int mpc_queue_size, double controller_sampling_time, double prediction_sampling_time);
  ~MPCQueue();

  // Each input point must use timestamp_ns as its absolute desired ROS time.
  // A replacement starts a new trajectory; a splice refreshes a rolling
  // horizon from its first covered timestamp onward.
  void insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue,
                                 bool replace_existing);

  void getQueue(int64_t now_ns,
                Vector3dDeque& position_reference,
                Vector3dDeque& velocity_reference,
                Vector3dDeque& acceleration_reference,
                std::deque<double>& yaw_reference,
                std::deque<double>& yaw_rate_reference);

 private:
  int mpc_queue_size_;
  const int maximum_queue_size_;

  double prediction_sampling_time_;
  double queue_dt_;

  mav_msgs::EigenTrajectoryPointDeque reference_points_;
  mav_msgs::EigenTrajectoryPoint last_output_;
  bool has_last_output_;

  void linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,
                                   mav_msgs::EigenTrajectoryPointDeque& output_queue) const;
  mav_msgs::EigenTrajectoryPoint sampleAt(int64_t timestamp_ns) const;
  void prunePastPoints(int64_t now_ns);
};

}  // namespace mav_control

#endif  // MAV_NONLINEAR_MPC_MPC_QUEUE_H_
