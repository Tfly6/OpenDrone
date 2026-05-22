#include <mav_linear_mpc/mpc_queue.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

#include <ros/ros.h>

namespace {
inline double shortestAngularDistance(const double from, const double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}
}  // namespace

namespace mav_control {

MPCQueue::MPCQueue(int mpc_queue_size,
                   double controller_sampling_time,
                   double prediction_sampling_time)
    : minimum_queue_size_(0),
      mpc_queue_size_(mpc_queue_size),
      maximum_queue_size_(10000),
      current_queue_size_(0),
      prediction_sampling_time_(prediction_sampling_time),
      queue_dt_(controller_sampling_time),
      queue_start_time_(0.0) {
  const int step_ratio = std::max(1, static_cast<int>(std::ceil(prediction_sampling_time_ / queue_dt_)));
  minimum_queue_size_ = std::max(2, mpc_queue_size_ * step_ratio);

  mav_msgs::EigenTrajectoryPoint point;
  fillQueueWithPoint(point);
}

MPCQueue::~MPCQueue() {}

void MPCQueue::clearQueue() {
  position_reference_.clear();
  velocity_reference_.clear();
  acceleration_reference_.clear();
  yaw_reference_.clear();
  yaw_rate_reference_.clear();
  queue_start_time_ = 0.0;
  current_queue_size_ = 0;
}

void MPCQueue::fillQueueWithPoint(const mav_msgs::EigenTrajectoryPoint& point) {
  while (current_queue_size_ < minimum_queue_size_) {
    pushBackPoint(point);
  }
}

void MPCQueue::insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue) {
  mav_msgs::EigenTrajectoryPointDeque interpolated_queue;
  linearInterpolateTrajectory(queue, interpolated_queue);
  if (interpolated_queue.empty()) {
    return;
  }

  const double commanded_time_from_start =
      static_cast<double>(interpolated_queue.begin()->time_from_start_ns) * 1e-9;

  if (commanded_time_from_start <= queue_start_time_ || commanded_time_from_start <= 1e-4) {
    clearQueue();
    queue_start_time_ = commanded_time_from_start;
  } else {
    const double queue_end_time = queue_start_time_ + current_queue_size_ * queue_dt_;
    if (commanded_time_from_start < queue_end_time) {
      const size_t start_index = std::round((commanded_time_from_start - queue_start_time_) / queue_dt_);
      position_reference_.erase(position_reference_.begin() + start_index, position_reference_.end());
      velocity_reference_.erase(velocity_reference_.begin() + start_index, velocity_reference_.end());
      acceleration_reference_.erase(acceleration_reference_.begin() + start_index, acceleration_reference_.end());
      yaw_reference_.erase(yaw_reference_.begin() + start_index, yaw_reference_.end());
      yaw_rate_reference_.erase(yaw_rate_reference_.begin() + start_index, yaw_rate_reference_.end());
      current_queue_size_ = static_cast<int>(start_index);
    }
  }

  for (auto it = interpolated_queue.begin(); it != interpolated_queue.end(); ++it) {
    position_reference_.push_back(it->position_W);
    velocity_reference_.push_back(it->velocity_W);
    acceleration_reference_.push_back(it->acceleration_W);
    yaw_reference_.push_back(it->getYaw());
    yaw_rate_reference_.push_back(it->getYawRate());
    current_queue_size_++;
  }

  if (current_queue_size_ < minimum_queue_size_) {
    fillQueueWithPoint(interpolated_queue.back());
  }
}

void MPCQueue::pushBackPoint(const mav_msgs::EigenTrajectoryPoint& point) {
  if (current_queue_size_ >= maximum_queue_size_) {
    ROS_WARN_STREAM_THROTTLE(1, "MPC: maximum queue size reached, discarding last reference point");
    return;
  }

  position_reference_.push_back(point.position_W);
  velocity_reference_.push_back(point.velocity_W);
  acceleration_reference_.push_back(point.acceleration_W);
  yaw_reference_.push_back(point.getYaw());
  yaw_rate_reference_.push_back(point.getYawRate());
  current_queue_size_++;
}

void MPCQueue::popFrontPoint() {
  if (current_queue_size_ <= 0) {
    return;
  }

  position_reference_.pop_front();
  velocity_reference_.pop_front();
  acceleration_reference_.pop_front();
  yaw_reference_.pop_front();
  yaw_rate_reference_.pop_front();
  queue_start_time_ += queue_dt_;
  current_queue_size_--;
}

void MPCQueue::getLastPoint(mav_msgs::EigenTrajectoryPoint* point) const {
  assert(point != nullptr);
  if (current_queue_size_ <= 0) {
    return;
  }

  point->position_W = position_reference_.back();
  point->velocity_W = velocity_reference_.back();
  point->acceleration_W = acceleration_reference_.back();
  point->setFromYaw(yaw_reference_.back());
  point->setFromYawRate(yaw_rate_reference_.back());
}

void MPCQueue::updateQueue() {
  popFrontPoint();
  if (current_queue_size_ <= 0) {
    return;
  }

  mav_msgs::EigenTrajectoryPoint point;
  getLastPoint(&point);

  while (current_queue_size_ < minimum_queue_size_) {
    pushBackPoint(point);
  }
}

void MPCQueue::getQueue(Vector3dDeque& position_reference,
                        Vector3dDeque& velocity_reference,
                        Vector3dDeque& acceleration_reference,
                        std::deque<double>& yaw_reference,
                        std::deque<double>& yaw_rate_reference) {
  position_reference.clear();
  velocity_reference.clear();
  acceleration_reference.clear();
  yaw_reference.clear();
  yaw_rate_reference.clear();

  if (current_queue_size_ <= 0) {
    return;
  }

  const int step = std::max(1, static_cast<int>(std::ceil(prediction_sampling_time_ / queue_dt_)));
  const int usable_size = step * static_cast<int>(std::floor(static_cast<double>(current_queue_size_) / step));

  for (int i = 0; i < usable_size; i += step) {
    position_reference.push_back(position_reference_.at(i));
    velocity_reference.push_back(velocity_reference_.at(i));
    acceleration_reference.push_back(acceleration_reference_.at(i));
    yaw_reference.push_back(yaw_reference_.at(i));
    yaw_rate_reference.push_back(yaw_rate_reference_.at(i));
  }
}

void MPCQueue::linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,
                                           mav_msgs::EigenTrajectoryPointDeque& interpolated_queue) const {
  interpolated_queue.clear();
  if (input_queue.empty()) {
    ROS_WARN_THROTTLE(1.0, "MPCQueue: Empty reference queue.");
    return;
  }

  if (input_queue.size() < 2) {
    const mav_msgs::EigenTrajectoryPoint& p0 = input_queue.front();
    const int horizon_points = std::max(2, minimum_queue_size_);
    const int64_t base_t = p0.time_from_start_ns;
    const int64_t dt_ns = static_cast<int64_t>(queue_dt_ * 1.0e9);
    const double yaw0 = p0.getYaw();
    const double yaw_rate0 = p0.getYawRate();

    for (int i = 0; i < horizon_points; ++i) {
      const double t = i * queue_dt_;
      mav_msgs::EigenTrajectoryPoint point;
      point.position_W = p0.position_W + p0.velocity_W * t + 0.5 * p0.acceleration_W * t * t;
      point.velocity_W = p0.velocity_W + p0.acceleration_W * t;
      point.acceleration_W = p0.acceleration_W;
      point.setFromYaw(yaw0 + yaw_rate0 * t);
      point.setFromYawRate(yaw_rate0);
      point.time_from_start_ns = base_t + static_cast<int64_t>(i) * dt_ns;
      interpolated_queue.push_back(point);
    }
    return;
  }

  std::vector<int64_t> time_input;
  std::vector<int64_t> time_output;

  const int64_t kDefaultDtNsec = 10000000;
  int64_t time_prev = 0;
  for (auto it = input_queue.begin(); it != input_queue.end(); ++it) {
    int64_t current_time = it->time_from_start_ns;
    if (it != input_queue.begin() && current_time == 0) {
      current_time = time_prev + kDefaultDtNsec;
    }
    time_prev = current_time;
    time_input.push_back(current_time);
  }

  const int64_t time_0 = time_input.front();
  const int64_t queue_dt_ns = static_cast<int64_t>(queue_dt_ * 1.0e9);
  time_output.push_back(time_0);

  const int sample_count = (time_input.back() - time_input.front()) / queue_dt_ns + 1;
  for (int i = 1; i < sample_count; ++i) {
    time_output.push_back(time_0 + queue_dt_ns * i);
  }

  for (auto it = time_output.begin(); it != time_output.end(); ++it) {
    mav_msgs::EigenTrajectoryPoint point;

    auto sol = std::upper_bound(time_input.begin(), time_input.end(), *it);
    if (sol == time_input.begin()) {
      ++sol;
    }
    if (sol == time_input.end()) {
      --sol;
    }

    const int64_t time1 = *(sol - 1);
    const int64_t time2 = *sol;
    if (time2 == time1) {
      continue;
    }

    const size_t idx2 = static_cast<size_t>(sol - time_input.begin());
    const size_t idx1 = idx2 - 1;

    const Eigen::Vector3d& position_2 = input_queue.at(idx2).position_W;
    const Eigen::Vector3d& position_1 = input_queue.at(idx1).position_W;
    point.position_W = position_1 + ((position_2 - position_1) / (time2 - time1)) * (*it - time1);

    const Eigen::Vector3d& velocity_2 = input_queue.at(idx2).velocity_W;
    const Eigen::Vector3d& velocity_1 = input_queue.at(idx1).velocity_W;
    point.velocity_W = velocity_1 + ((velocity_2 - velocity_1) / (time2 - time1)) * (*it - time1);

    const Eigen::Vector3d& acceleration_2 = input_queue.at(idx2).acceleration_W;
    const Eigen::Vector3d& acceleration_1 = input_queue.at(idx1).acceleration_W;
    point.acceleration_W = acceleration_1 + ((acceleration_2 - acceleration_1) / (time2 - time1)) * (*it - time1);

    const double yaw_1 = input_queue.at(idx1).getYaw();
    const double yaw_2 = input_queue.at(idx2).getYaw();
    const double ratio = static_cast<double>(*it - time1) / static_cast<double>(time2 - time1);
    point.setFromYaw(yaw_1 + shortestAngularDistance(yaw_1, yaw_2) * ratio);

    const double yaw_rate_1 = input_queue.at(idx1).getYawRate();
    const double yaw_rate_2 = input_queue.at(idx2).getYawRate();
    point.setFromYawRate(yaw_rate_1 + ((yaw_rate_2 - yaw_rate_1) / (time2 - time1)) * (*it - time1));

    point.time_from_start_ns = *it;
    interpolated_queue.push_back(point);
  }
}

}  // namespace mav_control
