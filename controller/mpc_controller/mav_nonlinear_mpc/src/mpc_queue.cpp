#include <mav_nonlinear_mpc/mpc_queue.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <ros/ros.h>

namespace {
inline double shortestAngularDistance(const double from, const double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

mav_msgs::EigenTrajectoryPoint Interpolate(
    const mav_msgs::EigenTrajectoryPoint& first,
    const mav_msgs::EigenTrajectoryPoint& second, const int64_t timestamp_ns) {
  const int64_t dt_ns = second.timestamp_ns - first.timestamp_ns;
  if (dt_ns <= 0) return second;
  const double ratio = static_cast<double>(timestamp_ns - first.timestamp_ns) /
                       static_cast<double>(dt_ns);
  mav_msgs::EigenTrajectoryPoint point;
  point.position_W = first.position_W + ratio * (second.position_W - first.position_W);
  point.velocity_W = first.velocity_W + ratio * (second.velocity_W - first.velocity_W);
  point.acceleration_W = first.acceleration_W + ratio * (second.acceleration_W - first.acceleration_W);
  point.setFromYaw(first.getYaw() + shortestAngularDistance(first.getYaw(), second.getYaw()) * ratio);
  point.setFromYawRate(first.getYawRate() + ratio * (second.getYawRate() - first.getYawRate()));
  point.timestamp_ns = timestamp_ns;
  return point;
}
}  // namespace

namespace mav_control {

MPCQueue::MPCQueue(int mpc_queue_size, double controller_sampling_time,
                   double prediction_sampling_time)
    : mpc_queue_size_(mpc_queue_size), maximum_queue_size_(10000),
      prediction_sampling_time_(prediction_sampling_time), queue_dt_(controller_sampling_time),
      has_last_output_(false) {}

MPCQueue::~MPCQueue() {}

void MPCQueue::insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue,
                                         const bool replace_existing) {
  mav_msgs::EigenTrajectoryPointDeque interpolated;
  linearInterpolateTrajectory(queue, interpolated);
  if (interpolated.empty()) return;
  if (replace_existing) {
    reference_points_.clear();
  } else {
    const int64_t first_ns = interpolated.front().timestamp_ns;
    const int64_t last_ns = interpolated.back().timestamp_ns;
    reference_points_.erase(std::remove_if(reference_points_.begin(), reference_points_.end(),
        [first_ns, last_ns](const mav_msgs::EigenTrajectoryPoint& point) {
          return point.timestamp_ns >= first_ns && point.timestamp_ns <= last_ns;
        }), reference_points_.end());
  }
  reference_points_.insert(reference_points_.end(), interpolated.begin(), interpolated.end());
  std::sort(reference_points_.begin(), reference_points_.end(),
      [](const mav_msgs::EigenTrajectoryPoint& lhs, const mav_msgs::EigenTrajectoryPoint& rhs) {
        return lhs.timestamp_ns < rhs.timestamp_ns;
      });
  reference_points_.erase(std::unique(reference_points_.begin(), reference_points_.end(),
      [](const mav_msgs::EigenTrajectoryPoint& lhs, const mav_msgs::EigenTrajectoryPoint& rhs) {
        return lhs.timestamp_ns == rhs.timestamp_ns;
      }), reference_points_.end());
  if (reference_points_.size() > static_cast<size_t>(maximum_queue_size_)) {
    reference_points_.erase(reference_points_.begin(), reference_points_.end() - maximum_queue_size_);
    ROS_WARN_STREAM_THROTTLE(1.0, "MPC: maximum timed reference buffer size reached");
  }
}

void MPCQueue::getQueue(const int64_t now_ns, Vector3dDeque& position_reference,
                        Vector3dDeque& velocity_reference,
                        Vector3dDeque& acceleration_reference,
                        std::deque<double>& yaw_reference,
                        std::deque<double>& yaw_rate_reference) {
  position_reference.clear(); velocity_reference.clear(); acceleration_reference.clear();
  yaw_reference.clear(); yaw_rate_reference.clear();
  const int64_t prediction_dt_ns = static_cast<int64_t>(prediction_sampling_time_ * 1.0e9);
  for (int i = 0; i < mpc_queue_size_; ++i) {
    const mav_msgs::EigenTrajectoryPoint point = sampleAt(now_ns + i * prediction_dt_ns);
    position_reference.push_back(point.position_W); velocity_reference.push_back(point.velocity_W);
    acceleration_reference.push_back(point.acceleration_W); yaw_reference.push_back(point.getYaw());
    yaw_rate_reference.push_back(point.getYawRate());
    if (i == 0) { last_output_ = point; has_last_output_ = true; }
  }
  prunePastPoints(now_ns);
}

mav_msgs::EigenTrajectoryPoint MPCQueue::sampleAt(const int64_t timestamp_ns) const {
  if (reference_points_.empty()) return has_last_output_ ? last_output_ : mav_msgs::EigenTrajectoryPoint();
  if (timestamp_ns < reference_points_.front().timestamp_ns) {
    return has_last_output_ ? last_output_ : reference_points_.front();
  }
  if (timestamp_ns >= reference_points_.back().timestamp_ns) return reference_points_.back();
  const auto second = std::upper_bound(reference_points_.begin(), reference_points_.end(), timestamp_ns,
      [](const int64_t timestamp, const mav_msgs::EigenTrajectoryPoint& point) {
        return timestamp < point.timestamp_ns;
      });
  const auto first = std::prev(second);
  const int64_t gap_ns = second->timestamp_ns - first->timestamp_ns;
  const int64_t max_contiguous_gap_ns = static_cast<int64_t>(1.5 * queue_dt_ * 1.0e9);
  if (gap_ns > max_contiguous_gap_ns) return *first;
  return Interpolate(*first, *second, timestamp_ns);
}

void MPCQueue::prunePastPoints(const int64_t now_ns) {
  while (reference_points_.size() > 1 && reference_points_[1].timestamp_ns <= now_ns) {
    reference_points_.pop_front();
  }
}

void MPCQueue::linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,
                                           mav_msgs::EigenTrajectoryPointDeque& output_queue) const {
  output_queue.clear();
  if (input_queue.empty()) { ROS_WARN_THROTTLE(1.0, "MPCQueue: empty reference trajectory."); return; }
  std::vector<mav_msgs::EigenTrajectoryPoint> input(input_queue.begin(), input_queue.end());
  std::sort(input.begin(), input.end(), [](const mav_msgs::EigenTrajectoryPoint& lhs,
                                           const mav_msgs::EigenTrajectoryPoint& rhs) {
    return lhs.timestamp_ns < rhs.timestamp_ns;
  });
  input.erase(std::unique(input.begin(), input.end(), [](const mav_msgs::EigenTrajectoryPoint& lhs,
                                                         const mav_msgs::EigenTrajectoryPoint& rhs) {
    return lhs.timestamp_ns == rhs.timestamp_ns;
  }), input.end());
  const int64_t queue_dt_ns = std::max<int64_t>(1, static_cast<int64_t>(queue_dt_ * 1.0e9));
  if (input.size() == 1) {
    const mav_msgs::EigenTrajectoryPoint& first = input.front();
    const int horizon_points = std::max(
        2, static_cast<int>(std::ceil(
               mpc_queue_size_ * prediction_sampling_time_ / queue_dt_)) + 1);
    for (int i = 0; i < horizon_points; ++i) {
      const double t = i * queue_dt_;
      mav_msgs::EigenTrajectoryPoint point;
      point.position_W = first.position_W + first.velocity_W * t + 0.5 * first.acceleration_W * t * t;
      point.velocity_W = first.velocity_W + first.acceleration_W * t;
      point.acceleration_W = first.acceleration_W;
      point.setFromYaw(first.getYaw() + first.getYawRate() * t);
      point.setFromYawRate(first.getYawRate()); point.timestamp_ns = first.timestamp_ns + i * queue_dt_ns;
      output_queue.push_back(point);
    }
    return;
  }
  const int64_t start_ns = input.front().timestamp_ns;
  const int64_t end_ns = input.back().timestamp_ns;
  for (int64_t timestamp_ns = start_ns; timestamp_ns < end_ns; timestamp_ns += queue_dt_ns) {
    const auto second = std::upper_bound(input.begin(), input.end(), timestamp_ns,
        [](const int64_t timestamp, const mav_msgs::EigenTrajectoryPoint& point) {
          return timestamp < point.timestamp_ns;
        });
    output_queue.push_back(Interpolate(*std::prev(second), *second, timestamp_ns));
  }
  output_queue.push_back(input.back());
}

}  // namespace mav_control
