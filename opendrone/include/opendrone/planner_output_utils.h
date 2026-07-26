#pragma once

#include <cmath>
#include <cstdint>

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <opendrone/PlannerOutput.h>
#include <opendrone/PlannerOutputPoint.h>

namespace opendrone {
namespace planner_output {

inline bool HasField(const PlannerOutputPoint& point, const uint32_t mask) {
  return (point.valid_mask & mask) != 0;
}

inline Eigen::Vector3d ToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

inline Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector) {
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

inline Eigen::Quaterniond QuaternionFromYaw(const double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline double SelectYaw(const PlannerOutputPoint& point, const double fallback_yaw) {
  return HasField(point, PlannerOutputPoint::VALID_YAW) ? point.yaw : fallback_yaw;
}

inline double SelectYawRate(const PlannerOutputPoint& point) {
  return HasField(point, PlannerOutputPoint::VALID_YAW_RATE) ? point.yaw_rate : 0.0;
}

inline Eigen::Vector3d SelectPosition(const PlannerOutputPoint& point,
                                      const Eigen::Vector3d& fallback = Eigen::Vector3d::Zero()) {
  return HasField(point, PlannerOutputPoint::VALID_POSITION) ? ToEigen(point.position) : fallback;
}

inline Eigen::Vector3d SelectVelocity(const PlannerOutputPoint& point,
                                      const Eigen::Vector3d& fallback = Eigen::Vector3d::Zero()) {
  return HasField(point, PlannerOutputPoint::VALID_VELOCITY) ? ToEigen(point.velocity) : fallback;
}

inline Eigen::Vector3d SelectAcceleration(const PlannerOutputPoint& point,
                                          const Eigen::Vector3d& fallback = Eigen::Vector3d::Zero()) {
  return HasField(point, PlannerOutputPoint::VALID_ACCELERATION) ? ToEigen(point.acceleration) : fallback;
}

inline Eigen::Vector3d SelectJerk(const PlannerOutputPoint& point,
                                  const Eigen::Vector3d& fallback = Eigen::Vector3d::Zero()) {
  return HasField(point, PlannerOutputPoint::VALID_JERK) ? ToEigen(point.jerk) : fallback;
}

inline Eigen::Vector3d SelectSnap(const PlannerOutputPoint& point,
                                  const Eigen::Vector3d& fallback = Eigen::Vector3d::Zero()) {
  return HasField(point, PlannerOutputPoint::VALID_SNAP) ? ToEigen(point.snap) : fallback;
}

inline Eigen::Vector3d SelectAngularVelocity(const PlannerOutputPoint& point,
                                             const double fallback_yaw_rate = 0.0) {
  if (HasField(point, PlannerOutputPoint::VALID_ANGULAR_VELOCITY)) {
    return ToEigen(point.angular_velocity);
  }
  return Eigen::Vector3d(0.0, 0.0, SelectYawRate(point) + fallback_yaw_rate);
}

inline int64_t ToNanoseconds(const ros::Duration& duration) {
  return static_cast<int64_t>(duration.sec) * 1000000000LL + static_cast<int64_t>(duration.nsec);
}

inline int64_t AbsoluteTimeNs(const PlannerOutput& output,
                              const PlannerOutputPoint& point) {
  return static_cast<int64_t>(output.trajectory_start_time.toNSec()) +
         ToNanoseconds(point.time_from_start);
}

}  // namespace planner_output
}  // namespace opendrone
