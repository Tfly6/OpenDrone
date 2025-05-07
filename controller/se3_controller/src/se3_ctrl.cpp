// ref: se3_example.cpp
#include "se3_controller/se3_ctrl.h"

se3Ctrl::se3Ctrl(const ros::NodeHandle &nh):nh_(nh)
{
    cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    desire_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/desire_odom_pub", 10);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_service_ = nh_.advertiseService("/land", &se3Ctrl::landCallback, this);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &se3Ctrl::OdomCallback, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &se3Ctrl::IMUCallback, this);
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &se3Ctrl::StateCallback, this);
    desire_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/desire_odom", 10, &se3Ctrl::DesireOdomCallback, this);
    multiDOFJoint_sub_ = nh_.subscribe("/command/trajectory", 10, &se3Ctrl::multiDOFJointCallback, this);

    exec_timer_ = nh_.createTimer(ros::Duration(0.01), &se3Ctrl::execFSMCallback, this);

    dynamic_tune_cb_type_ = boost::bind(&se3Ctrl::DynamicTuneCallback, this, _1, _2);
    dynamic_tune_server_.setCallback(dynamic_tune_cb_type_);

    nh_.param<bool>("enable_sim", sim_enable_, false);
    nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
    nh_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
    nh_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
    nh_.param<double>("geo_fence/z", geo_fence_[2], 4.0);

    enu_frame_ = true;
    vel_in_body_ = true;
    arm_triggered_ = false;
    offboard_triggered_ = false;

    init_pose_ << 0, 0, 0.5;
    node_state_ = WAITING_FOR_CONNECTED;

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
    desired_state_.p(2) = takeoff_height_;
    desired_state_.yaw = 0.0;

    se3_controller_.init(hover_percent_, max_hover_percent_, enu_frame_, vel_in_body_);
    se3_controller_.setup(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
                            kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
                            limit_err_p_, limit_err_v_, limit_err_a_,
                            limit_d_err_p_, limit_d_err_v_, limit_d_err_a_);
}


void se3Ctrl::execFSMCallback(const ros::TimerEvent &e){
    // exec_timer_.stop();

    // Controller_Output_t output;
    // if(se3_controller_.calControl(odom_data_, imu_data_, desired_state_, output)){
    //     send_cmd(output, true);
    //     desire_odom_pub_.publish(desire_odom_);
    //     if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true){
    //         se3_controller_.estimateTa(imu_data_.a);
    //     }
    // }
    
    // exec_timer_.start();
    switch (node_state_)
    {
    case WAITING_FOR_CONNECTED:{
        while(ros::ok() && !currState_.connected){
            ros::spinOnce();
        }
        
        ROS_INFO("connected!");
        node_state_ = WAITING_FOR_OFFBOARD;
        break;
    }
    case WAITING_FOR_OFFBOARD:{
        // cout <<"check: " <<currState_.mode <<endl;
        
        pubLocalPose(init_pose_);
        trigger_offboard();
        trigger_arm();
        if(currState_.mode == "OFFBOARD" && currState_.armed){
            ROS_INFO("Ready TakeOff");
            node_state_ = MISSION_EXECUTION;
            // last_ = ros::Time::now();
        }
        break;
    }
    // case TAKEOFF:{
        
    //     if(is_arrive(currPose_, targetPos_)){
    //         ROS_INFO("TakeOff Complete");
    //         node_state_ = MISSION_EXECUTION;
    //     }
    //     break;
    // }
        
    case MISSION_EXECUTION:{
        Controller_Output_t output;
        if(se3_controller_.calControl(odom_data_, imu_data_, desired_state_, output)){
            send_cmd(output, true);
            desire_odom_pub_.publish(desire_odom_);
            se3_controller_.estimateTa(imu_data_.a);
        }
        break;
    }
    case LANDING: {
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
            ROS_INFO("land enabled");
        }
        node_state_ = LANDED;
        // ros::spinOnce();
        break;
    }
    case LANDED:
        if(!currState_.armed){
            ROS_INFO("Landed. Please set to position control and disarm.");
            exec_timer_.stop();
        }
        // ros::spinOnce();
        break;
    default:
        break;
    }
}

void se3Ctrl::send_cmd(const Controller_Output_t &output, bool angle){
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

void se3Ctrl::pubLocalPose(const Eigen::Vector3d &pose) 
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];

    local_pos_pub_.publish(msg);
}

bool se3Ctrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    ROS_INFO("trigger land!");
    node_state_ = LANDING;
    return true;
}

void se3Ctrl::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_data_.feed(msg, enu_frame_, vel_in_body_);
    bool judge_x = ((odom_data_.p(0) >= geo_fence_[0]) || (odom_data_.p(0) <= -geo_fence_[0]));
    bool judge_y = ((odom_data_.p(1) >= geo_fence_[1]) || (odom_data_.p(1) <= -geo_fence_[1]));
    bool judge_z = (odom_data_.p(2) >= geo_fence_[2]);
    bool judge = (judge_x || judge_y || judge_z);
    if(judge && currState_.mode != mavros_msgs::State::MODE_PX4_LAND){
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = mavros_msgs::State::MODE_PX4_LAND;
        if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
            node_state_ = LANDED;
            ROS_WARN("obs Land enabled");
        }
    }
}

void se3Ctrl::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
    imu_data_.feed(msg, enu_frame_);
}

void se3Ctrl::StateCallback(const mavros_msgs::State::ConstPtr &msg){
    currState_ = *msg;
}

void se3Ctrl::DesireOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
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

void se3Ctrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) 
{
    // command/trajectory
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];

    desired_state_.p(0) = pt.transforms[0].translation.x;
    desired_state_.p(1) = pt.transforms[0].translation.y;
    desired_state_.p(2) = pt.transforms[0].translation.z;

    desired_state_.v(0) = pt.velocities[0].linear.x;
    desired_state_.v(1) = pt.velocities[0].linear.y;
    desired_state_.v(2) = pt.velocities[0].linear.z;

    desired_state_.a.setZero();
    desired_state_.j.setZero();

    desired_state_.q.w() = pt.transforms[0].rotation.w;
    desired_state_.q.x() = pt.transforms[0].rotation.x;
    desired_state_.q.y() = pt.transforms[0].rotation.y;
    desired_state_.q.z() = pt.transforms[0].rotation.z;

    desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
    desired_state_.yaw_rate = 0.0;
    // reference_request_last_ = reference_request_now_;
  
    // reference_request_now_ = ros::Time::now();
    // reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();
  
    // targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
    // targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;
  
    // targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;

    // Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
    //     pt.transforms[0].rotation.z);
    // Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    // yaw_ref_ = rpy(2);
}
