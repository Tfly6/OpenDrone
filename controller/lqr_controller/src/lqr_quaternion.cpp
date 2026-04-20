#include "lqr_controller/lqr_quaternion.hpp"

namespace lqr {

LQR_Quaternion::LQR_Quaternion(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
{
  // Initialize Q and R matrices with default values
  // Q: state weighting (position, quaternion, velocity) - 10 states
  Q_.setZero();
  Q_.diagonal() << 10, 10, 10,     // position x, y, z
                   5, 5, 5, 5,    // quaternion w, x, y, z
                   1, 1, 1;       // velocity x, y, z

  // R: control weighting (angular rates, thrust) - 4 controls
  R_.setZero();
  R_.diagonal() << 1, 1, 1, 0.1;  // wx, wy, wz, thrust

  initiated = false;
  callBack_ = ros::Time::now();
  init_time_ = ros::Time::now();
}

LQR_Quaternion::~LQR_Quaternion()
{
}

void LQR_Quaternion::setQ(const state_matrix_quat_t& Q)
{
  Q_ = Q;
}

void LQR_Quaternion::setR(const control_matrix_quat_t& R)
{
  R_ = R;
}

void LQR_Quaternion::setHoverReference(double x, double y, double z)
{
  // Set hover reference position (used when no trajectory is available)
  xref_.setZero();
  xref_(0) = x;  // target x
  xref_(1) = y;  // target y
  xref_(2) = z;  // target z
  // Use current orientation as reference
  xref_(3) = 1.0;
  xref_(4) = 0.0;
  xref_(5) = 0.0;
  xref_(6) = 0.0;
  // Zero velocity reference for hover
  xref_(7) = 0;
  xref_(8) = 0;
  xref_(9) = 0;
  uref_.setZero();
  uref_(3) = 9.81;  // Hover thrust
}

void LQR_Quaternion::setStates(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Extract states from odometry message
  position_enu_ << msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z;

  q_enu_.w() = msg->pose.pose.orientation.w;
  q_enu_.x() = msg->pose.pose.orientation.x;
  q_enu_.y() = msg->pose.pose.orientation.y;
  q_enu_.z() = msg->pose.pose.orientation.z;

  velocity_enu_ << msg->twist.twist.linear.x,
                   msg->twist.twist.linear.y,
                   msg->twist.twist.linear.z;
  velocity_enu_ = mavros::ftf::transform_frame_baselink_enu(velocity_enu_,q_enu_);

  /*Position*/
  x_(0) = position_enu_(0);
  x_(1) = position_enu_(1);
  x_(2) = position_enu_(2);

  /*Orientation*/
  x_(3) = q_enu_.w();
  x_(4) = q_enu_.x();
  x_(5) = q_enu_.y();
  x_(6) = q_enu_.z();

  /*Linear Velocities*/
  x_(7) = velocity_enu_(0);
  x_(8) = velocity_enu_(1);
  x_(9) = velocity_enu_(2);

  // Update reference based on trajectory if available
  if (!trajectory_.points.empty()) {
    setTrajectoryReference(xref_, uref_);
  }

  // Compute error
  setError(xref_,x_,xerror_);
}

void LQR_Quaternion::computeLQR()
{
  if ((ros::Time::now().toSec() - callBack_.toSec()) > 0.1)
  {
    A_ = A_quadrotor(xref_,uref_);
    B_ = B_quadrotor(xref_,uref_);

    if (lqrSolver_.compute(Q_, R_, A_, B_, Knew_))
    {
      if (!Knew_.hasNaN())
      {
        Kold_ = Knew_;
        // ROS_INFO("LQR Quaternion: solver converged in %zu iterations", lqrSolver_.getIterations());
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "LQR Quaternion: solver did not converge, K may be stale");
    }

    // Debug: print reference and some state info periodically
    // ROS_INFO_THROTTLE(2.0, "LQR Quat: xref [%.2f, %.2f, %.2f] uref [%.2f, %.2f, %.2f, %.2f]",
    //                   xref_(0), xref_(1), xref_(2), uref_(0), uref_(1), uref_(2), uref_(3));

    callBack_ = ros::Time::now();
  }
}

void LQR_Quaternion::setTrajectory(const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
  trajectory_ = msg;
  // ROS_INFO("LQR Controller: Received trajectory with %zu points", msg.points.size());
}

state_matrix_quat_t LQR_Quaternion::A_quadrotor(const state_vector_quat_t& x, const control_vector_quat_t& u)
{
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust  = u(3);
    Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
    Eigen::Matrix<double,4,4> q_partial_correction;
    Eigen::Matrix<double,4,4> dqdot_dq;
    Eigen::Matrix<double,3,4> dvdot_dq;
    Eigen::Matrix<double,4,1> q_vec;

    q_vec(0) = q.w();
    q_vec(1) = q.x();
    q_vec(2) = q.y();
    q_vec(3) = q.z();

    state_matrix_quat_t A;
    A.setZero();

    //Position
    A(0,7) = 1;
    A(1,8)= 1;
    A(2,9)= 1;
    Eigen::Matrix<double,4,4> Identity;

    //Orientation
    q_partial_correction = pow(q.norm(),-1.0)*(Identity.Identity() - pow(q.norm(),-2.0)*(q_vec * q_vec.transpose()));

    dqdot_dq << 0, -wx, -wy, -wz,
                wx, 0, wz, -wy,
                wy, -wz, 0, wx,
                wz, wy, -wx, 0;
    dqdot_dq = 0.5*dqdot_dq*q_partial_correction;

    A(3,3) = dqdot_dq(0,0);
    A(3,4) = dqdot_dq(0,1);
    A(3,5) = dqdot_dq(0,2);
    A(3,6) = dqdot_dq(0,3);

    A(4,3) = dqdot_dq(1,0);
    A(4,4) = dqdot_dq(1,1);
    A(4,5) = dqdot_dq(1,2);
    A(4,6) = dqdot_dq(1,3);

    A(5,3) = dqdot_dq(2,0);
    A(5,4) = dqdot_dq(2,1);
    A(5,5) = dqdot_dq(2,2);
    A(5,6) = dqdot_dq(2,3);

    A(6,3) = dqdot_dq(3,0);
    A(6,4) = dqdot_dq(3,1);
    A(6,5) = dqdot_dq(3,2);
    A(6,6) = dqdot_dq(3,3);


    //Velocity
    dvdot_dq << q.y(),  q.z(),  q.w(), q.x(),
              -q.x(), -q.w(),  q.z(), q.y(),
               q.w(), -q.x(), -q.y(), q.z();

    dvdot_dq = 2*norm_thrust*dvdot_dq*q_partial_correction;

    A(7,3) = dvdot_dq(0,0);
    A(7,4) = dvdot_dq(0,1);
    A(7,5) = dvdot_dq(0,2);
    A(7,6) = dvdot_dq(0,3);

    A(8,3) = dvdot_dq(1,0);
    A(8,4) = dvdot_dq(1,1);
    A(8,5) = dvdot_dq(1,2);
    A(8,6) = dvdot_dq(1,3);

    A(9,3) = dvdot_dq(2,0);
    A(9,4) = dvdot_dq(2,1);
    A(9,5) = dvdot_dq(2,2);
    A(9,6) = dvdot_dq(2,3);

    return A;
}

control_gain_matrix_quat_t LQR_Quaternion::B_quadrotor(const state_vector_quat_t& x, const control_vector_quat_t& u)
{
   double wx = u(0);
   double wy = u(1);
   double wz = u(2);
   double norm_thrust  = u(3);
   Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
   Eigen::Matrix<double,3,1> dvdot_dc;
   Eigen::Matrix<double,4,3> dqdot_dw;

   control_gain_matrix_quat_t B;
   B.setZero();

   dvdot_dc << 2*(q.w()*q.y() + q.x()*q.z()),
               2*(q.y()*q.z() - q.w()*q.x()),
               pow(q.w(),2) - pow(q.x(),2) - pow(q.y(),2) + pow(q.z(),2);

   B(7,3) = dvdot_dc(0);
   B(8,3) = dvdot_dc(1);
   B(9,3) = dvdot_dc(2);

   dqdot_dw << -q.x(), -q.y(), -q.z(),
                q.w(), -q.z(),  q.y(),
                q.z(),  q.w(), -q.x(),
               -q.y(),  q.x(),  q.w();

   dqdot_dw = 0.5*dqdot_dw;

   B(3,0) = dqdot_dw(0,0);
   B(3,1) = dqdot_dw(0,1);
   B(3,2) = dqdot_dw(0,2);

   B(4,0) = dqdot_dw(1,0);
   B(4,1) = dqdot_dw(1,1);
   B(4,2) = dqdot_dw(1,2);

   B(5,0) = dqdot_dw(2,0);
   B(5,1) = dqdot_dw(2,1);
   B(5,2) = dqdot_dw(2,2);

   B(6,0) = dqdot_dw(3,0);
   B(6,1) = dqdot_dw(3,1);
   B(6,2) = dqdot_dw(3,2);

   return B;
}

void LQR_Quaternion::setError(const state_vector_quat_t& xref, const state_vector_quat_t& x, state_vector_quat_t& xerror)
{
  /*Position error*/
  xerror(0) = (x(0) - xref(0));
  xerror(1) = (x(1) - xref(1));
  xerror(2) = (x(2) - xref(2));

  /*Orientation error*/
  Eigen::Quaterniond q(x(3),x(4),x(5),x(6));
  Eigen::Quaterniond qref(xref(3),xref(4),xref(5),xref(6));
  Eigen::Quaterniond qerror = q.inverse() * qref;
  xerror(3) = 0;
  xerror(4) = qerror.x();
  xerror(5) = qerror.y();
  xerror(6) = qerror.z();

  /*Velocity error*/
  xerror(7) = (x(7) - xref(7));
  xerror(8) = (x(8) - xref(8));
  xerror(9) = (x(9) - xref(9));
}

bool LQR_Quaternion::setTrajectoryReference(state_vector_quat_t& xref, control_vector_quat_t& uref)
{
  if (trajectory_.points.empty()) {
    return false;
  }

  // Find the trajectory point closest to current position
  int selected_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < static_cast<int>(trajectory_.points.size()); ++i) {
    const auto& pt = trajectory_.points[i];
    Eigen::Vector3d pt_pos(pt.transforms[0].translation.x,
                           pt.transforms[0].translation.y,
                           pt.transforms[0].translation.z);
    double dist = (position_enu_ - pt_pos).norm();

    if (dist < min_dist) {
      min_dist = dist;
      selected_idx = i;
    }
  }

  // Get the trajectory point
  const auto& pt = trajectory_.points[selected_idx];

  // Extract position
  xref(0) = pt.transforms[0].translation.x;
  xref(1) = pt.transforms[0].translation.y;
  xref(2) = pt.transforms[0].translation.z;

  // Extract velocity
  if (pt.velocities.size() > 0) {
    xref(7) = pt.velocities[0].linear.x;
    xref(8) = pt.velocities[0].linear.y;
    xref(9) = pt.velocities[0].linear.z;
  } else {
    xref(7) = 0;
    xref(8) = 0;
    xref(9) = 0;
  }

  // Extract orientation
  Eigen::Quaterniond q(pt.transforms[0].rotation.w,
                       pt.transforms[0].rotation.x,
                       pt.transforms[0].rotation.y,
                       pt.transforms[0].rotation.z);
  xref(3) = q.w();
  xref(4) = q.x();
  xref(5) = q.y();
  xref(6) = q.z();

  // Compute thrust from desired acceleration if available
  if (pt.accelerations.size() > 0) {
    Eigen::Vector3d accel(pt.accelerations[0].linear.x,
                          pt.accelerations[0].linear.y,
                          pt.accelerations[0].linear.z);
    Eigen::Vector3d thrust_dir = Eigen::Vector3d(0, 0, 9.81) + accel;
    uref(3) = thrust_dir.norm();
  } else {
    uref(3) = 9.81;  // Hover thrust
  }

  // Angular rates from trajectory (or zero if not available)
  if (pt.velocities.size() > 0) {
    Eigen::Vector3d ang_vel(pt.velocities[0].angular.x,
                            pt.velocities[0].angular.y,
                            pt.velocities[0].angular.z);
    // Transform angular velocity from world to body frame
    Eigen::Vector3d ang_vel_body = mavros::ftf::transform_frame_enu_baselink(ang_vel, q);
    uref(0) = ang_vel_body.x();
    uref(1) = ang_vel_body.y();
    uref(2) = ang_vel_body.z();
  } else {
    uref(0) = 0;
    uref(1) = 0;
    uref(2) = 0;
  }

  // Check if trajectory is finished
  if (selected_idx >= static_cast<int>(trajectory_.points.size()) - 1) {
    ROS_INFO("LQR Controller: Trajectory finished");
    return true;
  }

  return false;
}

Eigen::Vector3d LQR_Quaternion::quaternion_to_rpy_wrap(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d rpy;
  double roll = atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(pow(q.x(),2)+pow(q.y(),2)));
  double pitch = asin(2*(q.w()*q.y()-q.z()*q.x()));
  double yaw = atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(pow(q.y(),2)+pow(q.z(),2)));

  rpy << roll,
         pitch,
         yaw;

  return rpy;
}

state_vector_quat_t LQR_Quaternion::getError()
{
  return this->xerror_;
}

Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> LQR_Quaternion::getGain()
{
  return this->Kold_;
}

void LQR_Quaternion::setOutput(double output,int j)
{
  this->output_(j) = output;
}

void LQR_Quaternion::setOutput(control_vector_quat_t output)
{
  this->output_ = output;
}

control_vector_quat_t LQR_Quaternion::getOutput()
{
  return this->output_;
}

state_vector_quat_t LQR_Quaternion::getRefStates()
{
  return this->xref_;
}

control_vector_quat_t LQR_Quaternion::getTrajectoryControl()
{
  return this->uref_;
}

}/* namespace lqr */