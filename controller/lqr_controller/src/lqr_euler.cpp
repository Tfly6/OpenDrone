#include "lqr_controller/lqr_euler.hpp"

namespace lqr {

LQR_Euler::LQR_Euler(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
{
  // Initialize Q and R matrices with default values
  // Q: state weighting (position, orientation, velocity)
  Q_.setZero();
  Q_.diagonal() << 10, 10, 10,    // position x, y, z
                    5, 5, 10,     // orientation roll, pitch, yaw
                    1, 1, 1;      // velocity x, y, z

  // R: control weighting (angular rates, thrust)
  R_.setZero();
  R_.diagonal() << 1, 1, 1, 0.1;  // wx, wy, wz, thrust

  initiated = false;
  callBack_ = ros::Time::now();
  init_time_ = ros::Time::now();
}

LQR_Euler::~LQR_Euler()
{
}

void LQR_Euler::setQ(const state_matrix_t& Q)
{
  Q_ = Q;
}

void LQR_Euler::setR(const control_matrix_t& R)
{
  R_ = R;
}

void LQR_Euler::setHoverReference(double x, double y, double z)
{
  // Set hover reference position (used when no trajectory is available)
  xref_.setZero();
  xref_(0) = y;  // target x_ned
  xref_(1) = x;  // target y_ned
  xref_(2) = -z;  // target z_ned
  // Use current orientation as reference (roll, pitch, yaw)
  Eigen::Quaterniond q_ned = mavros::ftf::transform_orientation_baselink_aircraft(
      mavros::ftf::transform_orientation_enu_ned(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)));
  Eigen::Vector3d rpy = quaternion_to_rpy_wrap(q_ned);
  xref_(3) = rpy.x();  // roll
  xref_(4) = rpy.y();  // pitch
  xref_(5) = rpy.z();  // yaw
  // Zero velocity reference for hover
  xref_(6) = 0;
  xref_(7) = 0;
  xref_(8) = 0;
  uref_.setZero();
  uref_(3) = -9.81;  // Hover thrust
}

void LQR_Euler::setStates(const nav_msgs::Odometry::ConstPtr& msg)
{
  //Orientation
  q_enu_.w() = -1*msg->pose.pose.orientation.w;
  q_enu_.x() = -1*msg->pose.pose.orientation.x;
  q_enu_.y() = -1*msg->pose.pose.orientation.y;
  q_enu_.z() = -1*msg->pose.pose.orientation.z;

  q_ned_ = mavros::ftf::transform_orientation_enu_ned<Eigen::Quaterniond>(q_enu_);
  q_ned_ = mavros::ftf::transform_orientation_baselink_aircraft<Eigen::Quaterniond>(q_ned_);
  Eigen::Vector3d euler = quaternion_to_rpy_wrap(q_ned_);

  //position transformation
  position_enu_ << msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z;
  Eigen::Vector3d position_ned = mavros::ftf::transform_frame_enu_ned(position_enu_);

  //velocity transformation
  velocity_enu_ << msg->twist.twist.linear.x,
                   msg->twist.twist.linear.y,
                   msg->twist.twist.linear.z;
  Eigen::Vector3d velocity_ned = mavros::ftf::transform_frame_enu_ned(
      mavros::ftf::transform_frame_baselink_enu(velocity_enu_,q_enu_));

  //STATES
  //Position
  x_(0) = position_ned(0);
  x_(1) = position_ned(1);
  x_(2) = position_ned(2);

  //Orientation
  x_(3) = euler.x();
  x_(4) = euler.y();
  x_(5) = euler.z();

  //Linear Velocities
  x_(6) = velocity_ned(0);
  x_(7) = velocity_ned(1);
  x_(8) = velocity_ned(2);

  // Update reference based on trajectory if available
  if (!trajectory_.points.empty()) {
    setTrajectoryReference(xref_, uref_);
  }

  // Compute error
  setError(xref_,x_,xerror_);
}

void LQR_Euler::computeLQR()
{
  if ((ros::Time::now().toSec() - callBack_.toSec()) > 0.1)
  {
    A_ = A_quadrotor(x_,uref_);
    B_ = B_quadrotor(x_,uref_);

    if (lqrSolver_.compute(Q_, R_, A_, B_, Knew_))
    {
      if (!Knew_.hasNaN())
      {
        Kold_ = Knew_;
        // ROS_INFO("LQR Euler: solver converged in %zu iterations", lqrSolver_.getIterations());
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "LQR Euler: solver did not converge, K may be stale");
    }

    // Debug: print reference and some state info periodically
    // ROS_INFO_THROTTLE(2.0, "LQR Euler: xref [%.2f, %.2f, %.2f] uref [%.2f, %.2f, %.2f, %.2f]",
    //                   xref_(0), xref_(1), xref_(2), uref_(0), uref_(1), uref_(2), uref_(3));

    // std::cout << "A matrix:" << std::endl << A_ << std::endl;
    // std::cout << "B matrix:" << std::endl << B_ << std::endl;
    // std::cout << "LQR gain matrix:" << std::endl << Kold_ << std::endl;
    // std::cout << "Reference:" << std::endl << xref_ << std::endl;
    callBack_ = ros::Time::now();
  }
}

void LQR_Euler::setTrajectory(const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
  trajectory_ = msg;
  // ROS_INFO("LQR Euler Controller: Received trajectory with %zu points", msg.points.size());
}

state_matrix_t LQR_Euler::A_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust  = u(3);
    Eigen::Matrix<double,3,3> dEdot_dE;
    Eigen::Matrix<double,3,3> dvdot_dE;

    double phi = x(3);
    double theta = x(4);
    double psi = x(5);

    state_matrix_t A;
    A.setZero();

    //Position
    A(0,6) = 1;
    A(1,7)= 1;
    A(2,8)= 1;
    Eigen::Matrix<double,4,4> Identity;

    //Orientation
    dEdot_dE << wy*cos(phi)*tan(theta)-wz*sin(phi)*tan(theta), pow(1/cos(theta),2)*(wy*sin(phi)+wz*cos(phi)), 0,
                -wy*sin(phi)-wz*cos(phi), 0, 0,
                (1/cos(theta))*(wy*cos(phi)- wz*sin(phi)), (1/cos(theta))*tan(theta)*(wy*sin(phi)+wz*cos(phi)), 0;

    A(3,3) = dEdot_dE(0,0);
    A(3,4) = dEdot_dE(0,1);
    A(3,5) = dEdot_dE(0,2);

    A(4,3) = dEdot_dE(1,0);
    A(4,4) = dEdot_dE(1,1);
    A(4,5) = dEdot_dE(1,2);

    A(5,3) = dEdot_dE(2,0);
    A(5,4) = dEdot_dE(2,1);
    A(5,5) = dEdot_dE(2,2);

    //Velocity
    dvdot_dE << cos(phi)*sin(psi)-sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(theta)*cos(psi), sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi),
               -cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)*sin(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
               -sin(phi)*cos(theta),cos(phi)*sin(theta),sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
    dvdot_dE = -norm_thrust*dvdot_dE;

    A(6,3) = dvdot_dE(0,0);
    A(6,4) = dvdot_dE(0,1);
    A(6,5) = dvdot_dE(0,2);

    A(7,3) = dvdot_dE(1,0);
    A(7,4) = dvdot_dE(1,1);
    A(7,5) = dvdot_dE(1,2);

    A(8,3) = dvdot_dE(2,0);
    A(8,4) = dvdot_dE(2,1);
    A(8,5) = dvdot_dE(2,2);

    return A;
}

control_gain_matrix_t LQR_Euler::B_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
   double wx = u(0);
   double wy = u(1);
   double wz = u(2);
   double norm_thrust  = u(3);
   Eigen::Matrix<double,3,1> dvdot_dc;
   Eigen::Matrix<double,3,3> dEdot_dw;

   double phi = x(3);
   double theta = x(4);
   double psi = x(5);

   control_gain_matrix_t B;
   B.setZero();

   dvdot_dc << -(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)),
               -(-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)),
               -(cos(phi)*cos(theta));

   B(6,3) = dvdot_dc(0);
   B(7,3) = dvdot_dc(1);
   B(8,3) = dvdot_dc(2);

   dEdot_dw << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
               0, cos(phi), -sin(phi),
               0, sin(phi)*(1/cos(theta)), cos(phi)*(1/cos(theta));

   B(3,0) = dEdot_dw(0,0);
   B(3,1) = dEdot_dw(0,1);
   B(3,2) = dEdot_dw(0,2);

   B(4,0) = dEdot_dw(1,0);
   B(4,1) = dEdot_dw(1,1);
   B(4,2) = dEdot_dw(1,2);

   B(5,0) = dEdot_dw(2,0);
   B(5,1) = dEdot_dw(2,1);
   B(5,2) = dEdot_dw(2,2);

   return B;
}


void LQR_Euler::setError(const state_vector_t& xref, const state_vector_t& x, state_vector_t& xerror)
{
  //Position error
  xerror(0) = -1*(x(0) - xref(0));
  xerror(1) = -1*(x(1) - xref(1));
  xerror(2) = -1*(x(2) - xref(2));

  xerror(3) = (x(3) - xref(3));
  xerror(4) = (x(4) - xref(4));
  xerror(5) = (x(5) - xref(5));

  //Velocity error
  xerror(6) = -1*(x(6) - xref(6));
  xerror(7) = -1*(x(7) - xref(7));
  xerror(8) = -1*(x(8) - xref(8));
}

bool LQR_Euler::setTrajectoryReference(state_vector_t& xref, control_vector_t& uref)
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

  // Extract position (convert ENU to NED)
  Eigen::Vector3d pos_enu(pt.transforms[0].translation.x,
                           pt.transforms[0].translation.y,
                           pt.transforms[0].translation.z);
  Eigen::Vector3d pos_ned = mavros::ftf::transform_frame_enu_ned(pos_enu);
  xref(0) = pos_ned.x();
  xref(1) = pos_ned.y();
  xref(2) = pos_ned.z();

  // Extract orientation
  Eigen::Quaterniond q_enu(pt.transforms[0].rotation.w,
                           pt.transforms[0].rotation.x,
                           pt.transforms[0].rotation.y,
                           pt.transforms[0].rotation.z);
  Eigen::Quaterniond q_ned = mavros::ftf::transform_orientation_baselink_aircraft(
      mavros::ftf::transform_orientation_enu_ned(q_enu));
  Eigen::Vector3d rpy_ned = quaternion_to_rpy_wrap(q_ned);

  xref(3) = rpy_ned.x();
  xref(4) = rpy_ned.y();
  xref(5) = rpy_ned.z();

  // Extract velocity (convert ENU to NED)
  if (pt.velocities.size() > 0) {
    Eigen::Vector3d vel_enu(pt.velocities[0].linear.x,
                            pt.velocities[0].linear.y,
                            pt.velocities[0].linear.z);
    Eigen::Vector3d vel_ned = mavros::ftf::transform_frame_enu_ned(vel_enu);
    xref(6) = vel_ned.x();
    xref(7) = vel_ned.y();
    xref(8) = vel_ned.z();
  } else {
    xref(6) = 0;
    xref(7) = 0;
    xref(8) = 0;
  }

  // Compute thrust from desired acceleration if available
  if (pt.accelerations.size() > 0) {
    Eigen::Vector3d accel_enu(pt.accelerations[0].linear.x,
                              pt.accelerations[0].linear.y,
                              pt.accelerations[0].linear.z);
    Eigen::Vector3d thrust_dir = Eigen::Vector3d(0, 0, 9.81) + accel_enu;
    uref(3) = -thrust_dir.norm();
  } else {
    uref(3) = -9.81;  // Hover thrust
  }

  // Angular rates from trajectory (or zero if not available)
  if (pt.velocities.size() > 0) {
    Eigen::Vector3d ang_vel_enu(pt.velocities[0].angular.x,
                                 pt.velocities[0].angular.y,
                                 pt.velocities[0].angular.z);
    // Transform angular velocity from world to body frame
    Eigen::Vector3d ang_vel_body = mavros::ftf::transform_frame_enu_baselink(ang_vel_enu, q_enu);
    ang_vel_body = mavros::ftf::transform_frame_baselink_aircraft(ang_vel_body);
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
    ROS_INFO("LQR Euler Controller: Trajectory finished");
    return true;
  }

  return false;
}

Eigen::Vector3d LQR_Euler::quaternion_to_rpy_wrap(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d rpy;
  double roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()),1 - 2 * (pow(q.x(),2) + pow(q.y(),2)));
  double pitch = asin(2 * (q.w() * q.y() - q.z() * q.x()));
  double yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()),1 - 2 *(pow(q.y(),2) + pow(q.z(),2)));

  rpy << roll,
         pitch,
         yaw;

  return rpy;
}

state_vector_t LQR_Euler::getError()
{
  return this->xerror_;
}

Eigen::Matrix<double, nControls, nStates> LQR_Euler::getGain()
{
  return this->Kold_;
}

void LQR_Euler::setOutput(control_vector_t output)
{
  for (int i = 0; i < 3; i++) {
    if (output(i) > 2.0 || output(i) < -2.0) {
      if (output(i) > 2.0 ) {
        output(i) = 2.0;
      } else {
        output(i) = -2.0;
      }
    }
  }

  output(3) = -1*(output(3));
  this->output_ = output;
}

control_vector_t LQR_Euler::getOutput()
{
  return this->output_;
}

state_vector_t LQR_Euler::getRefStates()
{
  return this->xref_;
}

control_vector_t LQR_Euler::getTrajectoryControl()
{
  return this->uref_;
}

double LQR_Euler::getMotorCmd()
{
  double u;
  //Mapping
  u = LQR_Euler::output_(3)/17;
  return u;
}

}/* namespace lqr */