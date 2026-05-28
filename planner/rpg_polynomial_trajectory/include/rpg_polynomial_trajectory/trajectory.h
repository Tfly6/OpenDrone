#pragma once

#include <Eigen/Geometry>
#include <ros/ros.h>


namespace polynomial_trajectories {

struct TrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajectoryPoint() = default;

  ros::Duration time_from_start{0.0};

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};

  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};

  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_snap{Eigen::Vector3d::Zero()};

  double heading{0.0};
  double heading_rate{0.0};
  double heading_acceleration{0.0};
};

struct Trajectory {
  enum class TrajectoryType {
    UNDEFINED,
    GENERAL,
    ACCELERATION,
    JERK,
    SNAP
  };

  Trajectory() = default;
  explicit Trajectory(const TrajectoryPoint& point) {
    trajectory_type = TrajectoryType::GENERAL;
    points.push_back(point);
  }

  ros::Time timestamp{ros::Time::now()};
  TrajectoryType trajectory_type{TrajectoryType::UNDEFINED};

  std::list<TrajectoryPoint> points;
};

}  // namespace polynomial_trajectory