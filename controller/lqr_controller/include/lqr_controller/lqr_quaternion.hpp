#pragma once

#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <lqr_controller/declarations_quaternion.hpp>
#include <lqr_controller/lqr_solver.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <mavros/frame_tf.h>
#include <ros/package.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace lqr {
class LQR_Quaternion {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Quaternion(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~LQR_Quaternion();

    control_vector_quat_t getTrajectoryControl();
    state_vector_quat_t getError();
    Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> getGain();
    void setOutput(double output, int j);
    void setOutput(control_vector_quat_t output);
    control_vector_quat_t getOutput();
    state_vector_quat_t getRefStates();
    void setStates(const nav_msgs::Odometry::ConstPtr& msg);
    void setTrajectory(const trajectory_msgs::MultiDOFJointTrajectory& msg);
    void computeLQR();
    void setQ(const state_matrix_quat_t& Q);
    void setR(const control_matrix_quat_t& R);
    void setHoverReference(double x, double y, double z);

   private:

    void setError(const state_vector_quat_t& xref, const state_vector_quat_t& x, state_vector_quat_t& xerror);
    bool setTrajectoryReference(state_vector_quat_t& xref, control_vector_quat_t& uref);
    Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! State and control matrix dimensions
    const size_t state_dim = nStatesQuaternion;
    const size_t control_dim = nControlsQuaternion;

    // External trajectory storage
    trajectory_msgs::MultiDOFJointTrajectory trajectory_;
    bool initiated;

    ros::Time init_time_;
    Eigen::Vector3d position_enu_;
    Eigen::Vector3d velocity_enu_;
    Eigen::Quaterniond q_enu_;
    state_matrix_quat_t A_;
    control_gain_matrix_quat_t B_;
    Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> Kold_;
    Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> Knew_;
    ros::Time callBack_;
    state_vector_quat_t x_;
    control_vector_quat_t u_;
    state_vector_quat_t xref_;  // 10 states: [x, y, z, qw, qx, qy, qz, vx, vy, vz]
    control_vector_quat_t uref_; // 4 controls: [roll_rate, pitch_rate, yaw_rate, thrust]
    state_vector_quat_t xerror_;
    control_vector_quat_t output_;

    state_matrix_quat_t Q_;
    control_matrix_quat_t R_;
    LQRSolver<nStatesQuaternion, nControlsQuaternion> lqrSolver_;
    //states
    state_matrix_quat_t A_quadrotor(const state_vector_quat_t& x, const control_vector_quat_t& u);
    control_gain_matrix_quat_t B_quadrotor(const state_vector_quat_t& x, const control_vector_quat_t& u);
  };

} /* namespace lqr */