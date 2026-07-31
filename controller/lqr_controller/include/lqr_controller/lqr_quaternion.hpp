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
#include <opendrone/PlannerOutput.h>
#include <opendrone/PlannerOutputPoint.h>

namespace lqr {
class LQR_Quaternion {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Quaternion(ros::NodeHandle& privateNodeHandle);

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
    raw_state_vector_quat_t getRefStates();
    void setStates(const nav_msgs::Odometry::ConstPtr& msg);
    void setTrajectory(const opendrone::PlannerOutput& msg);
    void computeLQR();
    void setQ(const state_matrix_quat_t& Q);
    void setR(const control_matrix_quat_t& R);
    void setHoverReference(double x, double y, double z);

   private:

    void setError(const raw_state_vector_quat_t& xref,
                  const raw_state_vector_quat_t& x,
                  state_vector_quat_t& xerror);
    bool setTrajectoryReference(raw_state_vector_quat_t& xref, control_vector_quat_t& uref);
    int selectTrajectoryReferenceIndex() const;
    Eigen::Vector3d referenceAngularVelocityBody(
        const opendrone::PlannerOutputPoint& point,
        const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& thrust_direction,
        double thrust_norm,
        double yaw) const;
    static Eigen::Matrix3d hat(const Eigen::Vector3d& vector);
    static Eigen::Vector3d rotationVector(const Eigen::Quaterniond& quaternion);
    Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q);

    //! ROS node handle.
    ros::NodeHandle& privateNodeHandle_;

    //! State and control matrix dimensions
    const size_t state_dim = nStatesQuaternion;
    const size_t control_dim = nControlsQuaternion;

    // External trajectory storage
    opendrone::PlannerOutput trajectory_;
    bool initiated;
    bool haveState_{false};
    bool useSpatialReference_{true};
    double gainUpdatePeriodSec_{0.1};
    size_t lastSpatialReferenceIndex_{0};
    uint64_t spatialReferenceTrajectoryId_{0};

    ros::Time init_time_;
    Eigen::Vector3d position_enu_;
    Eigen::Vector3d velocity_enu_;
    Eigen::Quaterniond q_enu_;
    state_matrix_quat_t A_;
    control_gain_matrix_quat_t B_;
    Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> Kold_;
    Eigen::Matrix<double, nControlsQuaternion, nStatesQuaternion> Knew_;
    ros::Time callBack_;
    // Raw state/reference: [x, y, z, qw, qx, qy, qz, vx, vy, vz].
    raw_state_vector_quat_t x_;
    control_vector_quat_t u_;
    raw_state_vector_quat_t xref_;
    control_vector_quat_t uref_; // 4 controls: [roll_rate, pitch_rate, yaw_rate, thrust]
    state_vector_quat_t xerror_;
    control_vector_quat_t output_;

    state_matrix_quat_t Q_;
    control_matrix_quat_t R_;
    LQRSolver<nStatesQuaternion, nControlsQuaternion> lqrSolver_;
    //states
    state_matrix_quat_t A_quadrotor(const raw_state_vector_quat_t& x,
                                    const control_vector_quat_t& u);
    control_gain_matrix_quat_t B_quadrotor(const raw_state_vector_quat_t& x,
                                           const control_vector_quat_t& u);
  };

} /* namespace lqr */
