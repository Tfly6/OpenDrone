#include "lqr_controller/lqr_quaternion.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>

#include "opendrone/planner_output_utils.h"

namespace lqr {
namespace {

constexpr double kGravity = 9.81;
constexpr double kSmallNumber = 1.0e-8;

Eigen::Quaterniond QuaternionFromRawState(const raw_state_vector_quat_t& state) {
  Eigen::Quaterniond quaternion(state(3), state(4), state(5), state(6));
  if (quaternion.norm() < kSmallNumber || !quaternion.coeffs().allFinite()) {
    return Eigen::Quaterniond::Identity();
  }
  quaternion.normalize();
  return quaternion;
}

}  // namespace

LQR_Quaternion::LQR_Quaternion(ros::NodeHandle& privateNodeHandle)
    : privateNodeHandle_(privateNodeHandle) {
  // State is [position error, SO(3) rotation-vector error, velocity error].
  Q_.setZero();
  Q_.diagonal() << 10, 10, 10, 1, 1, 5, 1, 1, 1;

  R_.setZero();
  R_.diagonal() << 1, 1, 1, 0.1;

  std::string reference_selection("auto");
  privateNodeHandle_.param<std::string>("reference_selection", reference_selection,
                                        reference_selection);
  if (reference_selection == "temporal") {
    useSpatialReference_ = false;
  } else if (reference_selection == "auto" || reference_selection == "spatial") {
    useSpatialReference_ = true;
  } else {
    ROS_WARN("LQR: unknown reference_selection='%s'; using auto", reference_selection.c_str());
  }
  double lqr_update_rate = 10.0;
  privateNodeHandle_.param<double>("lqr_update_rate", lqr_update_rate, lqr_update_rate);
  lqr_update_rate = std::max(1.0, lqr_update_rate);
  gainUpdatePeriodSec_ = 1.0 / lqr_update_rate;

  initiated = false;
  x_.setZero();
  xref_.setZero();
  x_(3) = 1.0;
  xref_(3) = 1.0;
  u_.setZero();
  uref_.setZero();
  uref_(3) = kGravity;
  xerror_.setZero();
  output_.setZero();
  Kold_.setZero();
  Knew_.setZero();
  callBack_ = ros::Time::now();
  init_time_ = ros::Time::now();

  ROS_INFO("LQR: SO(3) error-state model enabled; gain update %.1f Hz; reference selection=%s%s",
           lqr_update_rate,
           reference_selection.c_str(),
           useSpatialReference_ ? " (spatial for complete trajectories, temporal for horizons)"
                                : " (temporal)");
}

LQR_Quaternion::~LQR_Quaternion() {}

void LQR_Quaternion::setQ(const state_matrix_quat_t& Q) {
  Q_ = Q;
}

void LQR_Quaternion::setR(const control_matrix_quat_t& R) {
  R_ = R;
}

void LQR_Quaternion::setHoverReference(double x, double y, double z) {
  xref_.setZero();
  xref_(0) = x;
  xref_(1) = y;
  xref_(2) = z;
  const Eigen::Quaterniond reference_quaternion =
      haveState_ ? q_enu_ : Eigen::Quaterniond::Identity();
  xref_(3) = reference_quaternion.w();
  xref_(4) = reference_quaternion.x();
  xref_(5) = reference_quaternion.y();
  xref_(6) = reference_quaternion.z();
  uref_.setZero();
  uref_(3) = kGravity;
  if (haveState_) {
    setError(xref_, x_, xerror_);
  }
}

void LQR_Quaternion::setStates(const nav_msgs::Odometry::ConstPtr& msg) {
  position_enu_ << msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z;

  q_enu_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                               msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z);
  if (q_enu_.norm() < kSmallNumber || !q_enu_.coeffs().allFinite()) {
    ROS_WARN_THROTTLE(1.0, "LQR: received invalid odometry orientation; using identity");
    q_enu_ = Eigen::Quaterniond::Identity();
  } else {
    q_enu_.normalize();
  }

  velocity_enu_ << msg->twist.twist.linear.x,
                   msg->twist.twist.linear.y,
                   msg->twist.twist.linear.z;
  velocity_enu_ = mavros::ftf::transform_frame_baselink_enu(velocity_enu_, q_enu_);

  x_(0) = position_enu_(0);
  x_(1) = position_enu_(1);
  x_(2) = position_enu_(2);
  x_(3) = q_enu_.w();
  x_(4) = q_enu_.x();
  x_(5) = q_enu_.y();
  x_(6) = q_enu_.z();
  x_(7) = velocity_enu_(0);
  x_(8) = velocity_enu_(1);
  x_(9) = velocity_enu_(2);
  haveState_ = true;

  if (!trajectory_.points.empty()) {
    setTrajectoryReference(xref_, uref_);
  }
  setError(xref_, x_, xerror_);
}

void LQR_Quaternion::computeLQR() {
  if (!haveState_ || (ros::Time::now() - callBack_).toSec() <= gainUpdatePeriodSec_) {
    return;
  }

  // State-dependent LQR: linearize the nonlinear model at the latest state
  // estimate, while retaining the trajectory input as the feed-forward term.
  A_ = A_quadrotor(x_, uref_);
  B_ = B_quadrotor(x_, uref_);
  if (lqrSolver_.compute(Q_, R_, A_, B_, Knew_)) {
    if (Knew_.allFinite()) {
      Kold_ = Knew_;
    }
  } else {
    ROS_WARN_THROTTLE(5.0, "LQR: Riccati solver did not converge; retaining previous gain");
  }
  callBack_ = ros::Time::now();
}

void LQR_Quaternion::setTrajectory(const opendrone::PlannerOutput& msg) {
  if (msg.trajectory_id != spatialReferenceTrajectoryId_) {
    lastSpatialReferenceIndex_ = 0;
    spatialReferenceTrajectoryId_ = msg.trajectory_id;
  }
  trajectory_ = msg;
}

Eigen::Matrix3d LQR_Quaternion::hat(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
            vector.z(), 0.0, -vector.x(),
            -vector.y(), vector.x(), 0.0;
  return result;
}

Eigen::Vector3d LQR_Quaternion::rotationVector(const Eigen::Quaterniond& input) {
  Eigen::Quaterniond quaternion = input;
  if (quaternion.norm() < kSmallNumber || !quaternion.coeffs().allFinite()) {
    return Eigen::Vector3d::Zero();
  }
  quaternion.normalize();
  // q and -q describe the same rotation.  Select the shortest SO(3) error.
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::Vector3d imaginary = quaternion.vec();
  const double imaginary_norm = imaginary.norm();
  if (imaginary_norm < kSmallNumber) {
    return 2.0 * imaginary;
  }
  const double angle = 2.0 * std::atan2(imaginary_norm, quaternion.w());
  return angle * imaginary / imaginary_norm;
}

state_matrix_quat_t LQR_Quaternion::A_quadrotor(const raw_state_vector_quat_t& x,
                                                 const control_vector_quat_t& u) {
  const Eigen::Quaterniond quaternion = QuaternionFromRawState(x);
  const Eigen::Vector3d body_z = quaternion.toRotationMatrix().col(2);

  state_matrix_quat_t A;
  A.setZero();
  A.block<3, 3>(0, 6).setIdentity();
  // δ(R e3) = δθ x (R e3) = -hat(R e3) δθ for a world-frame SO(3) error.
  A.block<3, 3>(6, 3) = -u(3) * hat(body_z);
  return A;
}

control_gain_matrix_quat_t LQR_Quaternion::B_quadrotor(
    const raw_state_vector_quat_t& x, const control_vector_quat_t&) {
  const Eigen::Quaterniond quaternion = QuaternionFromRawState(x);
  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  control_gain_matrix_quat_t B;
  B.setZero();
  // Body-rate commands live in the body frame; the SO(3) error is world-frame.
  B.block<3, 3>(3, 0) = rotation;
  B.block<3, 1>(6, 3) = rotation.col(2);
  return B;
}

void LQR_Quaternion::setError(const raw_state_vector_quat_t& xref,
                               const raw_state_vector_quat_t& x,
                               state_vector_quat_t& xerror) {
  xerror.segment<3>(0) = x.segment<3>(0) - xref.segment<3>(0);

  const Eigen::Quaterniond current = QuaternionFromRawState(x);
  const Eigen::Quaterniond reference = QuaternionFromRawState(xref);
  // Left-invariant error R R_ref^T is expressed in world coordinates, exactly
  // the coordinates used by the translational dynamics and the Jacobian.
  xerror.segment<3>(3) = rotationVector(current * reference.conjugate());
  xerror.segment<3>(6) = x.segment<3>(7) - xref.segment<3>(7);
}

int LQR_Quaternion::selectTrajectoryReferenceIndex() const {
  if (trajectory_.points.empty()) {
    return -1;
  }

  // A horizon only contains present/future samples, so timestamp selection is
  // required.  Complete trajectories use the paper's spatial projection.
  if (trajectory_.is_horizon || !useSpatialReference_) {
    ros::Time reference_time = trajectory_.trajectory_start_time;
    if (reference_time.isZero()) {
      reference_time = trajectory_.header.stamp;
    }
    ros::Duration target_time = ros::Time::now() - reference_time;
    if (target_time.toSec() < 0.0) {
      target_time = ros::Duration(0.0);
    }

    const int64_t target_ns = opendrone::planner_output::ToNanoseconds(target_time);
    int selected_idx = static_cast<int>(trajectory_.points.size()) - 1;
    int64_t best_error_ns = std::numeric_limits<int64_t>::max();
    for (int i = 0; i < static_cast<int>(trajectory_.points.size()); ++i) {
      const int64_t point_ns =
          opendrone::planner_output::ToNanoseconds(trajectory_.points[i].time_from_start);
      const int64_t error_ns = std::llabs(point_ns - target_ns);
      if (error_ns < best_error_ns) {
        best_error_ns = error_ns;
        selected_idx = i;
      }
      if (point_ns >= target_ns) {
        break;
      }
    }
    return selected_idx;
  }

  const size_t first_index = std::min(lastSpatialReferenceIndex_, trajectory_.points.size() - 1);
  int selected_idx = static_cast<int>(first_index);
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (size_t i = first_index; i < trajectory_.points.size(); ++i) {
    const Eigen::Vector3d position =
        opendrone::planner_output::SelectPosition(trajectory_.points[i], position_enu_);
    const double distance_squared = (position - position_enu_).squaredNorm();
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      selected_idx = static_cast<int>(i);
    }
  }
  return selected_idx;
}

Eigen::Vector3d LQR_Quaternion::referenceAngularVelocityBody(
    const opendrone::PlannerOutputPoint& point,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& thrust_direction,
    const double thrust_norm,
    const double yaw) const {
  if (opendrone::planner_output::HasField(
          point, opendrone::PlannerOutputPoint::VALID_ANGULAR_VELOCITY)) {
    return rotation.transpose() * opendrone::planner_output::SelectAngularVelocity(point);
  }

  if (opendrone::planner_output::HasField(point, opendrone::PlannerOutputPoint::VALID_JERK) &&
      thrust_norm > kSmallNumber) {
    const Eigen::Vector3d jerk = opendrone::planner_output::SelectJerk(point);
    const Eigen::Vector3d thrust_direction_dot =
        (Eigen::Matrix3d::Identity() - thrust_direction * thrust_direction.transpose()) * jerk /
        thrust_norm;
    const double yaw_rate = opendrone::planner_output::SelectYawRate(point);
    const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d heading_dot(-std::sin(yaw) * yaw_rate,
                                      std::cos(yaw) * yaw_rate, 0.0);
    const Eigen::Vector3d cross = thrust_direction.cross(heading);
    if (cross.norm() > kSmallNumber) {
      const Eigen::Vector3d body_y = rotation.col(1);
      const Eigen::Vector3d cross_dot =
          thrust_direction_dot.cross(heading) + thrust_direction.cross(heading_dot);
      const Eigen::Vector3d body_y_dot =
          (Eigen::Matrix3d::Identity() - body_y * body_y.transpose()) * cross_dot / cross.norm();
      const Eigen::Vector3d body_x_dot =
          body_y_dot.cross(thrust_direction) + body_y.cross(thrust_direction_dot);
      Eigen::Matrix3d rotation_dot;
      rotation_dot.col(0) = body_x_dot;
      rotation_dot.col(1) = body_y_dot;
      rotation_dot.col(2) = thrust_direction_dot;
      Eigen::Matrix3d omega_hat = rotation.transpose() * rotation_dot;
      omega_hat = 0.5 * (omega_hat - omega_hat.transpose());
      return Eigen::Vector3d(omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0));
    }
  }

  // PlannerOutput yaw_rate is expressed in ENU/world coordinates.
  return rotation.transpose() * Eigen::Vector3d(0.0, 0.0,
                                                  opendrone::planner_output::SelectYawRate(point));
}

bool LQR_Quaternion::setTrajectoryReference(raw_state_vector_quat_t& xref,
                                             control_vector_quat_t& uref) {
  const int selected_idx = selectTrajectoryReferenceIndex();
  if (selected_idx < 0) {
    return false;
  }
  if (!trajectory_.is_horizon && useSpatialReference_) {
    lastSpatialReferenceIndex_ = static_cast<size_t>(selected_idx);
  }
  const auto& point = trajectory_.points[static_cast<size_t>(selected_idx)];

  const Eigen::Vector3d position = opendrone::planner_output::SelectPosition(point, position_enu_);
  xref.segment<3>(0) = position;
  xref.segment<3>(7) = opendrone::planner_output::SelectVelocity(point);

  const Eigen::Vector3d acceleration = opendrone::planner_output::SelectAcceleration(point);
  const Eigen::Vector3d thrust = Eigen::Vector3d(0.0, 0.0, kGravity) + acceleration;
  const double thrust_norm = thrust.norm();
  Eigen::Vector3d body_z = Eigen::Vector3d::UnitZ();
  if (thrust_norm > kSmallNumber) {
    body_z = thrust / thrust_norm;
  }
  const double yaw = opendrone::planner_output::SelectYaw(point, quaternion_to_rpy_wrap(q_enu_).z());
  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d body_y = body_z.cross(heading);
  if (body_y.norm() < kSmallNumber) {
    body_y = body_z.cross(Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0.0));
  }
  body_y.normalize();
  const Eigen::Vector3d body_x = body_y.cross(body_z).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y;
  rotation.col(2) = body_z;
  Eigen::Quaterniond reference(rotation);
  reference.normalize();
  const Eigen::Quaterniond previous_reference = QuaternionFromRawState(xref);
  if (previous_reference.coeffs().dot(reference.coeffs()) < 0.0) {
    reference.coeffs() *= -1.0;
  }
  xref(3) = reference.w();
  xref(4) = reference.x();
  xref(5) = reference.y();
  xref(6) = reference.z();

  uref(3) = thrust_norm;
  uref.head<3>() = referenceAngularVelocityBody(point, rotation, body_z, thrust_norm, yaw);
  return true;
}

Eigen::Vector3d LQR_Quaternion::quaternion_to_rpy_wrap(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  return Eigen::Vector3d(std::atan2(rotation(2, 1), rotation(2, 2)),
                         std::asin(std::max(-1.0, std::min(1.0, -rotation(2, 0)))),
                         std::atan2(rotation(1, 0), rotation(0, 0)));
}

state_vector_quat_t LQR_Quaternion::getError() { return xerror_; }

Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> LQR_Quaternion::getGain() {
  return Kold_;
}

void LQR_Quaternion::setOutput(double output, int j) { output_(j) = output; }

void LQR_Quaternion::setOutput(control_vector_quat_t output) { output_ = output; }

control_vector_quat_t LQR_Quaternion::getOutput() { return output_; }

raw_state_vector_quat_t LQR_Quaternion::getRefStates() { return xref_; }

control_vector_quat_t LQR_Quaternion::getTrajectoryControl() { return uref_; }

}  // namespace lqr
