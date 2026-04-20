#pragma once

#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <lqr_controller/declarations_euler.hpp>
#include <lqr_controller/lqr_solver.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <mavros/frame_tf.h>
#include <ros/package.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace lqr {
class LQR_Euler {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Euler(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~LQR_Euler();

    control_vector_t getTrajectoryControl();
    state_vector_t getError();
    Eigen::Matrix<double, nControls, nStates> getGain();
    void setOutput(control_vector_t output);
    control_vector_t getOutput();
    state_vector_t getRefStates();
    double getMotorCmd();
    void setStates(const nav_msgs::Odometry::ConstPtr& msg);
    void setTrajectory(const trajectory_msgs::MultiDOFJointTrajectory& msg);
    void computeLQR();
    void setQ(const state_matrix_t& Q);
    void setR(const control_matrix_t& R);
    void setHoverReference(double x, double y, double z);

   private:

    void setError(const state_vector_t& xref, const state_vector_t& x, state_vector_t& xerror);
    bool setTrajectoryReference(state_vector_t& xref, control_vector_t& uref);
    Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond& q);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! State and control matrix dimensions
    const size_t state_dim = nStates;
    const size_t control_dim = nControls;

    // External trajectory storage
    trajectory_msgs::MultiDOFJointTrajectory trajectory_;
    bool initiated;

    ros::Time init_time_;
    Eigen::Vector3d position_enu_;
    Eigen::Vector3d velocity_enu_;
    Eigen::Quaterniond q_enu_;
    Eigen::Quaterniond q_ned_;
    state_matrix_t A_;
    control_gain_matrix_t B_;
    Eigen::Matrix<double, nControls, nStates> Kold_;
    Eigen::Matrix<double, nControls, nStates> Knew_;
    ros::Time callBack_;
    state_vector_t x_;
    control_vector_t u_;
    state_vector_t xref_; // 9 states: [x, y, z, roll, pitch, yaw, vx, vy, vz]
    control_vector_t uref_; // 4 controls: [roll_rate, pitch_rate, yaw_rate, thrust]
    state_vector_t xerror_;
    control_vector_t output_;

    state_matrix_t Q_;
    control_matrix_t R_;
    LQRSolver<nStates, nControls> lqrSolver_;

    state_matrix_t A_quadrotor(const state_vector_t& x, const control_vector_t& u);
    control_gain_matrix_t B_quadrotor(const state_vector_t& x, const control_vector_t& u);
  };

} /* namespace */