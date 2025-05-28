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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "se3_controller/se3_controller.hpp"
#include "se3_controller/se3_dynamic_tuneConfig.h"
// #include "math_utils/math_utils.h"
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

using namespace std;

class se3Ctrl{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_, desire_odom_pub_, local_pos_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_;
    ros::Subscriber desire_odom_sub_, desire_angle_sub_, multiDOFJoint_sub_;
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
    SE3_CONTROLLER se3_controller_;

    bool sim_enable_, arm_triggered_{false}, offboard_triggered_{false}, takeoffFlag_{false};
    double takeoff_height_;
    Eigen::Vector3d init_pose_, geo_fence_;;

    Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
    double limit_err_p_, limit_err_v_, limit_err_a_, limit_d_err_p_, limit_d_err_v_, limit_d_err_a_;
    double hover_percent_, max_hover_percent_;
    bool enu_frame_, vel_in_body_;

    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig> dynamic_tune_server_;
    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig>::CallbackType dynamic_tune_cb_type_;

    enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED } node_state_;

    void execFSMCallback(const ros::TimerEvent &e);

    void send_cmd(const Controller_Output_t &output, bool angle);
    void pubLocalPose(const Eigen::Vector3d &pose); 

    bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void StateCallback(const mavros_msgs::State::ConstPtr &msg);
    void DesireOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);


    void DynamicTuneCallback(se3_controller::se3_dynamic_tuneConfig &config, uint32_t level){
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

        ROS_INFO("desire posit: %f %f %f", config.desire_px, config.desire_py, config.desire_pz);
        ROS_INFO("desire euler: %f %f %f", config.desire_roll, config.desire_pitch, config.desire_yaw);

        desired_state_.p(0) = config.desire_px;
        desired_state_.p(1) = config.desire_py;
        desired_state_.p(2) = config.desire_pz;

        desired_state_.v.setZero();
        desired_state_.a.setZero();
        desired_state_.j.setZero();

        Eigen::Quaterniond q = utils::euler2quat(config.desire_roll, config.desire_pitch, config.desire_yaw);
        desired_state_.q.w() = q.w();
        desired_state_.q.x() = q.x();
        desired_state_.q.y() = q.y();
        desired_state_.q.z() = q.z();

        desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
        desired_state_.yaw_rate = 0.0;

        desire_odom_.pose.pose.position.x = desired_state_.p(0);
        desire_odom_.pose.pose.position.y = desired_state_.p(1);
        desire_odom_.pose.pose.position.z = desired_state_.p(2);

        desire_odom_.twist.twist.linear.x = desired_state_.v(0);
        desire_odom_.twist.twist.linear.y = desired_state_.v(1);
        desire_odom_.twist.twist.linear.z = desired_state_.v(2);

        desire_odom_.pose.pose.orientation.w = desired_state_.q.w();
        desire_odom_.pose.pose.orientation.x = desired_state_.q.x();
        desire_odom_.pose.pose.orientation.y = desired_state_.q.y();
        desire_odom_.pose.pose.orientation.z = desired_state_.q.z();
        
        se3_controller_.setup(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
                                kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
                                limit_err_p_, limit_err_v_, limit_err_a_,
                                limit_d_err_p_, limit_d_err_v_, limit_d_err_a_);



        printf("\n");
    }

    


public:
    se3Ctrl(const ros::NodeHandle &nh);
    ~se3Ctrl(){};

    void trigger_offboard()
    {
        if (sim_enable_) {
            // Enable OFFBoard mode and arm automatically
            // This will only run if the vehicle is simulated
            
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if (currState_.mode != "OFFBOARD" && !offboard_triggered_) {
                cout <<"check1: " <<currState_.mode <<endl;
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    offboard_triggered_ = true;
                    ROS_INFO("Offboard enabled");
                }
            } 
        }
        else {
            ROS_WARN("Not in sim,please be careful!");
            if (currState_.mode != "OFFBOARD") {
                ROS_INFO("Switch To Offboard Mode");
            }
        }
    }

    void trigger_arm()
    {
        // mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if( currState_.mode == "OFFBOARD"){
            if( !currState_.armed && !arm_triggered_){
                if( arming_client_.call(arm_cmd) &&arm_cmd.response.success){
                    arm_triggered_ = true;
                    ROS_INFO("Vehicle armed");
                }
                // cout << "check1"<< arming_client_.call(arm_cmd)<<endl;
            }
        }
    }
};

#endif