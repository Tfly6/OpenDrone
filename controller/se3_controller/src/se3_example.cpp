#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <dynamic_reconfigure/server.h>

#include "se3_controller/se3_controller.hpp"
#include "se3_controller/se3_dynamic_tuneConfig.h"
#include <std_msgs/Float64.h>

class SE3_EXAMPLE{
private:
    ros::NodeHandle node_;
    ros::Publisher cmd_pub_, desire_odom_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_, desire_odom_sub_, desire_angle_sub_;
    ros::ServiceClient set_mode_client_;
    ros::Timer exec_timer_;
    mavros_msgs::State state_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Desired_State_t desired_state_;
    SE3_CONTROLLER se3_controller_;
    nav_msgs::Odometry desire_odom_;
    Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
    double limit_err_p_, limit_err_v_, limit_err_a_, limit_d_err_p_, limit_d_err_v_, limit_d_err_a_;
    double hover_percent_, max_hover_percent_;
    bool enu_frame_, vel_in_body_;

    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig> dynamic_tune_server_;
    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig>::CallbackType dynamic_tune_cb_type_;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        odom_data_.feed(msg, enu_frame_, vel_in_body_);
        bool judge_x = ((odom_data_.p(0) >= 1.3) || (odom_data_.p(0) <= -1.3));
        bool judge_y = ((odom_data_.p(1) >= 2.3) || (odom_data_.p(1) <= -2.3));
        bool judge_z = (odom_data_.p(2) >= 1.5);
        bool judge = (judge_x || judge_y || judge_z);
        if(judge && state_.mode != mavros_msgs::State::MODE_PX4_LAND){
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = mavros_msgs::State::MODE_PX4_LAND;
        if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
            ROS_WARN("obs Land enabled");
        }
    }
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
        imu_data_.feed(msg, enu_frame_);
    }

    void StateCallback(const mavros_msgs::State::ConstPtr &msg){
        state_ = *msg;
    }

    void DesireOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        desire_odom_ = *msg;

        desired_state_.p(0) = msg->pose.pose.position.x;
        desired_state_.p(1) = msg->pose.pose.position.y;
        desired_state_.p(2) = msg->pose.pose.position.z;

        desired_state_.v(0) = msg->twist.twist.linear.x;
        desired_state_.v(1) = msg->twist.twist.linear.y;
        desired_state_.v(2) = msg->twist.twist.linear.z;

        desired_state_.a.setZero();
        desired_state_.j.setZero();

        desired_state_.q.w() = msg->pose.pose.orientation.w;
        desired_state_.q.x() = msg->pose.pose.orientation.x;
        desired_state_.q.y() = msg->pose.pose.orientation.y;
        desired_state_.q.z() = msg->pose.pose.orientation.z;

        desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
        desired_state_.yaw_rate = 0.0;
    }

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

    void send_cmd(const Controller_Output_t &output, bool angle){
        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.body_rate.x = output.bodyrates(0);
        cmd.body_rate.y = output.bodyrates(1);
        cmd.body_rate.z = output.bodyrates(2);
        cmd.orientation.w = output.q.w();
        cmd.orientation.x = output.q.x();
        cmd.orientation.y = output.q.y();
        cmd.orientation.z = output.q.z();
        cmd.thrust = output.thrust;
        if(angle){
            cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE + 
                            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE + 
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        }else{
            cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        }
        cmd_pub_.publish(cmd);
    }

    void execFSMCallback(const ros::TimerEvent &e){
        exec_timer_.stop();

        Controller_Output_t output;
        if(se3_controller_.calControl(odom_data_, imu_data_, desired_state_, output)){
            send_cmd(output, true);
            desire_odom_pub_.publish(desire_odom_);
            if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true){
                se3_controller_.estimateTa(imu_data_.a);
            }
        }
        
        exec_timer_.start();
    }
public:
    SE3_EXAMPLE(){};
    ~SE3_EXAMPLE(){};
    void init(ros::NodeHandle &nh){
        enu_frame_ = true;
        vel_in_body_ = true;

        cmd_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        desire_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/desire_odom_pub", 10);

        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 10, &SE3_EXAMPLE::OdomCallback, this);
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &SE3_EXAMPLE::OdomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &SE3_EXAMPLE::IMUCallback, this);
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &SE3_EXAMPLE::StateCallback, this);
        desire_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/desire_odom", 10, &SE3_EXAMPLE::DesireOdomCallback, this);

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &SE3_EXAMPLE::execFSMCallback, this);

        dynamic_tune_cb_type_ = boost::bind(&SE3_EXAMPLE::DynamicTuneCallback, this, _1, _2);
        dynamic_tune_server_.setCallback(dynamic_tune_cb_type_);

        kp_p_ << 0.85, 0.85, 1.5;
        kp_v_ << 1.5, 1.5, 1.5;
        kp_a_ << 1.5, 1.5, 1.5;
        kp_q_ << 5.5, 5.5, 0.1;
        kp_w_ << 1.5, 1.5, 0.1;

        kd_p_ << 0.1, 0.1, 0.0;
        kd_v_ << 0.0, 0.0, 0.0;
        kd_a_ << 0.0, 0.0, 0.0;
        kd_q_ << 0.0, 0.0, 0.0;
        kd_w_ << 0.0, 0.0, 0.0;

        limit_err_p_ = 3.0;
		limit_err_v_ = 2.0;
		limit_err_a_ = 1.0;
		limit_d_err_p_ = 3.5;
		limit_d_err_v_ = 1.0;
		limit_d_err_a_ = 1.0;

        hover_percent_ = 0.25;
        max_hover_percent_ = 0.75;

        desired_state_.p(0) = 0.0;
        desired_state_.p(1) = 0.0;
        desired_state_.p(2) = 0.3;
        desired_state_.yaw = 0.0;

        se3_controller_.init(hover_percent_, max_hover_percent_, enu_frame_, vel_in_body_);
        se3_controller_.setup(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
                                kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
                                limit_err_p_, limit_err_v_, limit_err_a_,
                                limit_d_err_p_, limit_d_err_v_, limit_d_err_a_);
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "px4_example_node");
    ros::NodeHandle nh("~");

    SE3_EXAMPLE se3_example;
    se3_example.init(nh);

    ros::spin();

    return 0;
}