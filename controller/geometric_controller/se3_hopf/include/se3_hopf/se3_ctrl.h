// ref: se3_example.cpp
#ifndef SE3_CTRL_H
#define SE3_CTRL_H
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "se3_hopf/se3_hopf.hpp"
#include "se3_hopf/se3_dynamic_tuneConfig.h"
// #include "math_utils/math_utils.h"
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>

using namespace std;

class Se3HopfCtrl{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_, local_pos_pub_, flight_state_pub_, reference_pose_pub_,
                   reference_vel_pub_, reference_acc_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_, multiDOFJoint_sub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceServer land_service_;
    ros::Timer exec_timer_;

    mavros_msgs::State currState_;
    mavros_msgs::CommandBool arm_cmd;
    nav_msgs::Odometry desire_odom_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Desired_State_t desired_state_;
    SE3_HOPF_CONTROLLER se3_hopf_;

    bool sim_enable_;
    bool arm_triggered_{false};
    bool offboard_triggered_{false};
    bool auto_takeoff_{false};
    // bool takeoffFlag_{false};
    bool landing_locked_{false};
    bool enable_auto_offboard_{false};
    bool enable_auto_arm_{false};
    int offboard_warmup_counter_{0};
    int offboard_warmup_count_{80};
    double request_interval_{1.0};
    ros::Time last_mode_request_;
    ros::Time last_arm_request_;
    double takeoff_height_;
    Eigen::Vector3d init_pose_, geo_fence_;;

    Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
    double limit_err_p_, limit_err_v_, limit_err_a_, limit_d_err_p_, limit_d_err_v_, limit_d_err_a_;
    double hover_percent_, max_hover_percent_;
    bool enu_frame_, vel_in_body_;

    dynamic_reconfigure::Server<se3_hopf::se3_dynamic_tuneConfig> dynamic_tune_server_;
    dynamic_reconfigure::Server<se3_hopf::se3_dynamic_tuneConfig>::CallbackType dynamic_tune_cb_type_;

    enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED, EMERGENCY } flightState_, prev_flightState_;

    std::string state2string(FlightState state) {
        switch (state) {
        case WAITING_FOR_CONNECTED:
            return "WAITING_FOR_CONNECTED";
        case WAITING_FOR_OFFBOARD:
            return "WAITING_FOR_OFFBOARD";
        case MISSION_EXECUTION:
            return "MISSION_EXECUTION";
        case LANDING:
            return "LANDING";
        case LANDED:
            return "LANDED";
        case TAKEOFF:
            return "TAKEOFF";
        case EMERGENCY:
            return "EMERGENCY";
        default:
            return "UNKNOWN_STATE";
        }
    }

    void execFSMCallback(const ros::TimerEvent &e);

    void send_cmd(const Controller_Output_t &output, bool angle);
    void pubLocalPose(const Eigen::Vector3d &pose); 

    bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void StateCallback(const mavros_msgs::State::ConstPtr &msg);
    void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg);
    void TrySetOffboard(const ros::Time &now);
    void TryArm(const ros::Time &now);


    void DynamicTuneCallback(se3_hopf::se3_dynamic_tuneConfig &config, uint32_t level){
        ROS_INFO("kp_p: %f %f %f", config.kp_px, config.kp_py, config.kp_pz);
        ROS_INFO("kp_v: %f %f %f", config.kp_vx, config.kp_vy, config.kp_vz);
        ROS_INFO("kp_a: %f %f %f", config.kp_ax, config.kp_ay, config.kp_az);
        ROS_INFO("kp_q: %f %f %f", config.kp_qx, config.kp_qy, config.kp_qz);
        ROS_INFO("kp_w: %f %f %f", config.kp_wx, config.kp_wy, config.kp_wz);

        ROS_INFO("kd_p: %f %f %f", config.kd_px, config.kd_py, config.kd_pz);
        ROS_INFO("kd_v: %f %f %f", config.kd_vx, config.kd_vy, config.kd_vz);
        ROS_INFO("kd_a: %f %f %f", config.kd_ax, config.kd_ay, config.kd_az);
        ROS_INFO("kd_q: %f %f %f", config.kd_qx, config.kd_qy, config.kd_qz);
        ROS_INFO("kd_w: %f %f %f", config.kd_wx, config.kd_wy, config.kd_wz);

        ROS_INFO("limit err   p v a: %f %f %f", config.limit_err_p, config.limit_err_v, config.limit_err_a);
        ROS_INFO("limit d err p v a: %f %f %f", config.limit_d_err_p, config.limit_d_err_v, config.limit_d_err_a);

        kp_p_ << config.kp_px, config.kp_py, config.kp_pz;
        kp_v_ << config.kp_vx, config.kp_vy, config.kp_vz;
        kp_a_ << config.kp_ax, config.kp_ay, config.kp_az;
        kp_q_ << config.kp_qx, config.kp_qy, config.kp_qz;
        kp_w_ << config.kp_wx, config.kp_wy, config.kp_wz;

        kd_p_ << config.kd_px, config.kd_py, config.kd_pz;
        kd_v_ << config.kd_vx, config.kd_vy, config.kd_vz;
        kd_a_ << config.kd_ax, config.kd_ay, config.kd_az;
        kd_q_ << config.kd_qx, config.kd_qy, config.kd_qz;
        kd_w_ << config.kd_wx, config.kd_wy, config.kd_wz;

        limit_err_p_ = config.limit_err_p;
		limit_err_v_ = config.limit_err_v;
		limit_err_a_ = config.limit_err_a;
		limit_d_err_p_ = config.limit_d_err_p;
		limit_d_err_v_ = config.limit_d_err_v;
		limit_d_err_a_ = config.limit_d_err_a;

        // ROS_INFO("desire posit: %f %f %f", config.desire_px, config.desire_py, config.desire_pz);
        // ROS_INFO("desire euler: %f %f %f", config.desire_roll, config.desire_pitch, config.desire_yaw);

        // desired_state_.p(0) = config.desire_px;
        // desired_state_.p(1) = config.desire_py;
        // desired_state_.p(2) = config.desire_pz;

        // desired_state_.v.setZero();
        // desired_state_.a.setZero();
        // desired_state_.j.setZero();

        // Eigen::Quaterniond q = utils::euler2quat(config.desire_roll, config.desire_pitch, config.desire_yaw);
        // desired_state_.q.w() = q.w();
        // desired_state_.q.x() = q.x();
        // desired_state_.q.y() = q.y();
        // desired_state_.q.z() = q.z();

        // desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
        // desired_state_.yaw_rate = 0.0;

        // desire_odom_.pose.pose.position.x = desired_state_.p(0);
        // desire_odom_.pose.pose.position.y = desired_state_.p(1);
        // desire_odom_.pose.pose.position.z = desired_state_.p(2);

        // desire_odom_.twist.twist.linear.x = desired_state_.v(0);
        // desire_odom_.twist.twist.linear.y = desired_state_.v(1);
        // desire_odom_.twist.twist.linear.z = desired_state_.v(2);

        // desire_odom_.pose.pose.orientation.w = desired_state_.q.w();
        // desire_odom_.pose.pose.orientation.x = desired_state_.q.x();
        // desire_odom_.pose.pose.orientation.y = desired_state_.q.y();
        // desire_odom_.pose.pose.orientation.z = desired_state_.q.z();
        
        se3_hopf_.setup(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
                                kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
                                limit_err_p_, limit_err_v_, limit_err_a_,
                                limit_d_err_p_, limit_d_err_v_, limit_d_err_a_);



        printf("\n");
    }

    


public:
    Se3HopfCtrl(const ros::NodeHandle &nh);
    ~Se3HopfCtrl(){};
};

#endif