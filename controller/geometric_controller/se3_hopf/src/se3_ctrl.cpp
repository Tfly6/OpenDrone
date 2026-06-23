// ref: se3_example.cpp
#include "se3_hopf/se3_ctrl.h"

Se3HopfCtrl::Se3HopfCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh), dynamic_tune_server_(private_nh)
{
    cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    reference_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/controller/reference_pose", 10);
    reference_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/controller/reference_velocity", 10);
    reference_acc_pub_ = nh_.advertise<geometry_msgs::AccelStamped>("/controller/reference_accel", 10);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_service_ = nh_.advertiseService("/land", &Se3HopfCtrl::landCallback, this);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &Se3HopfCtrl::OdomCallback, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &Se3HopfCtrl::IMUCallback, this);
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &Se3HopfCtrl::StateCallback, this);
    multiDOFJoint_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 10, &Se3HopfCtrl::multiDOFJointCallback, this);

    exec_timer_ = nh_.createTimer(ros::Duration(0.01), &Se3HopfCtrl::execFSMCallback, this);

    flight_state_pub_ = nh_.advertise<std_msgs::Int8>("/flight_state", 10);

    private_nh_.param<bool>("enable_sim", sim_enable_, false);
    private_nh_.param<bool>("enable_auto_offboard", enable_auto_offboard_, sim_enable_);
    private_nh_.param<bool>("enable_auto_arm", enable_auto_arm_, sim_enable_);
    private_nh_.param<bool>("use_dynamic_reconfigure", use_dynamic_reconfigure_, false);
    private_nh_.param<int>("offboard_warmup_count", offboard_warmup_count_, 80);
    private_nh_.param<double>("request_interval", request_interval_, 1.0);
    private_nh_.param<bool>("auto_takeoff", auto_takeoff_, true);
    private_nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
    private_nh_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
    private_nh_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
    private_nh_.param<double>("geo_fence/z", geo_fence_[2], 4.0);

    enu_frame_ = true;
    vel_in_body_ = true;
    // arm_triggered_ = false;
    // offboard_triggered_ = false;

    init_pose_ << 0, 0, 0.5;
    flightState_ = WAITING_FOR_CONNECTED;
    prev_flightState_ = flightState_;
    if (offboard_warmup_count_ < 1) {
        offboard_warmup_count_ = 1;
    }
    if (request_interval_ < 0.1) {
        request_interval_ = 0.1;
    }
    last_mode_request_ = ros::Time(0);
    last_arm_request_ = ros::Time(0);

    // kp_p_ << 0.85, 0.85, 1.5;
    // kp_v_ << 1.5, 1.5, 1.5;
    // kp_a_ << 1.5, 1.5, 1.5;
    // kp_q_ << 5.5, 5.5, 0.1;
    // kp_w_ << 1.5, 1.5, 0.1;

    // kd_p_ << 0.1, 0.1, 0.0;
    // kd_v_ << 0.0, 0.0, 0.0;
    // kd_a_ << 0.0, 0.0, 0.0;
    // kd_q_ << 0.0, 0.0, 0.0;
    // kd_w_ << 0.0, 0.0, 0.0;

    // limit_err_p_ = 3.0;
    // limit_err_v_ = 2.0;
    // limit_err_a_ = 1.0;
    // limit_d_err_p_ = 3.5;
    // limit_d_err_v_ = 1.0;
    // limit_d_err_a_ = 1.0;

    hover_percent_ = 0.25;
    max_hover_percent_ = 0.75;

    // desired_state_.p(0) = 0.0;
    // desired_state_.p(1) = 0.0;
    // desired_state_.p(2) = takeoff_height_;
    // desired_state_.yaw = 0.0;

    se3_hopf_.init(hover_percent_, max_hover_percent_, enu_frame_, vel_in_body_);
    if (use_dynamic_reconfigure_) {
        dynamic_tune_cb_type_ = boost::bind(&Se3HopfCtrl::DynamicTuneCallback, this, _1, _2);
        dynamic_tune_server_.setCallback(dynamic_tune_cb_type_);
        ROS_INFO("se3_hopf using dynamic_reconfigure for tuning parameters.");
    } else {
        loadStaticTuneConfig();
        ROS_INFO("se3_hopf using static ROS parameters for tuning parameters.");
    }
}

void Se3HopfCtrl::applyTuneConfig(const se3_hopf::se3_dynamic_tuneConfig &config) {
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

    se3_hopf_.setup(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
                    kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
                    limit_err_p_, limit_err_v_, limit_err_a_,
                    limit_d_err_p_, limit_d_err_v_, limit_d_err_a_);
}

void Se3HopfCtrl::loadStaticTuneConfig() {
    se3_hopf::se3_dynamic_tuneConfig config;
    private_nh_.param("kp_px", config.kp_px, 0.85);
    private_nh_.param("kp_py", config.kp_py, 0.85);
    private_nh_.param("kp_pz", config.kp_pz, 1.5);
    private_nh_.param("kp_vx", config.kp_vx, 1.5);
    private_nh_.param("kp_vy", config.kp_vy, 1.5);
    private_nh_.param("kp_vz", config.kp_vz, 1.5);
    private_nh_.param("kp_ax", config.kp_ax, 1.5);
    private_nh_.param("kp_ay", config.kp_ay, 1.5);
    private_nh_.param("kp_az", config.kp_az, 1.5);
    private_nh_.param("kp_qx", config.kp_qx, 5.5);
    private_nh_.param("kp_qy", config.kp_qy, 5.5);
    private_nh_.param("kp_qz", config.kp_qz, 0.1);
    private_nh_.param("kp_wx", config.kp_wx, 1.5);
    private_nh_.param("kp_wy", config.kp_wy, 1.5);
    private_nh_.param("kp_wz", config.kp_wz, 0.1);
    private_nh_.param("kd_px", config.kd_px, 0.1);
    private_nh_.param("kd_py", config.kd_py, 0.1);
    private_nh_.param("kd_pz", config.kd_pz, 0.0);
    private_nh_.param("kd_vx", config.kd_vx, 0.0);
    private_nh_.param("kd_vy", config.kd_vy, 0.0);
    private_nh_.param("kd_vz", config.kd_vz, 0.0);
    private_nh_.param("kd_ax", config.kd_ax, 0.0);
    private_nh_.param("kd_ay", config.kd_ay, 0.0);
    private_nh_.param("kd_az", config.kd_az, 0.0);
    private_nh_.param("kd_qx", config.kd_qx, 0.0);
    private_nh_.param("kd_qy", config.kd_qy, 0.0);
    private_nh_.param("kd_qz", config.kd_qz, 0.0);
    private_nh_.param("kd_wx", config.kd_wx, 0.0);
    private_nh_.param("kd_wy", config.kd_wy, 0.0);
    private_nh_.param("kd_wz", config.kd_wz, 0.0);
    private_nh_.param("limit_err_p", config.limit_err_p, 3.0);
    private_nh_.param("limit_err_v", config.limit_err_v, 2.0);
    private_nh_.param("limit_err_a", config.limit_err_a, 1.0);
    private_nh_.param("limit_d_err_p", config.limit_d_err_p, 3.5);
    private_nh_.param("limit_d_err_v", config.limit_d_err_v, 1.0);
    private_nh_.param("limit_d_err_a", config.limit_d_err_a, 1.0);
    applyTuneConfig(config);
}


void Se3HopfCtrl::execFSMCallback(const ros::TimerEvent &e){
    // exec_timer_.stop();

    // Controller_Output_t output;
    // if(se3_hopf_.calControl(odom_data_, imu_data_, desired_state_, output)){
    //     send_cmd(output, true);
    //     desire_odom_pub_.publish(desire_odom_);
    //     if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true){
    //         se3_hopf_.estimateTa(imu_data_.a);
    //     }
    // }
    
    // exec_timer_.start();
    std_msgs::Int8 flight_state_msg;
    flight_state_msg.data = static_cast<int8_t>(flightState_);
    flight_state_pub_.publish(flight_state_msg);

    geometry_msgs::PoseStamped ref_msg;
    ref_msg.header.stamp = ros::Time::now();
    ref_msg.header.frame_id = "map";
    ref_msg.pose.position.x = desired_state_.p(0);
    ref_msg.pose.position.y = desired_state_.p(1);
    ref_msg.pose.position.z = desired_state_.p(2);
    ref_msg.pose.orientation.w = 1.0;
    reference_pose_pub_.publish(ref_msg);

    geometry_msgs::TwistStamped ref_vel_msg;
    ref_vel_msg.header = ref_msg.header;
    ref_vel_msg.twist.linear.x = desired_state_.v(0);
    ref_vel_msg.twist.linear.y = desired_state_.v(1);
    ref_vel_msg.twist.linear.z = desired_state_.v(2);
    reference_vel_pub_.publish(ref_vel_msg);

    geometry_msgs::AccelStamped ref_acc_msg;
    ref_acc_msg.header = ref_msg.header;
    ref_acc_msg.accel.linear.x = desired_state_.a(0);
    ref_acc_msg.accel.linear.y = desired_state_.a(1);
    ref_acc_msg.accel.linear.z = desired_state_.a(2);
    reference_acc_pub_.publish(ref_acc_msg);
    
    if (flightState_ != prev_flightState_) {
        ROS_WARN_STREAM("State changed from " << state2string(prev_flightState_) << " to " << state2string(flightState_));
        prev_flightState_ = flightState_;
    }
    switch (flightState_)
    {
    case WAITING_FOR_CONNECTED:{
        ROS_INFO_ONCE("Waiting for FCU connection...");
        if(currState_.connected){
            ROS_INFO("connected!");
            offboard_warmup_counter_ = 0;
            flightState_ = WAITING_FOR_OFFBOARD;
        }
        break;
    }
    case WAITING_FOR_OFFBOARD:{
        ROS_INFO_ONCE("Waiting for OFFBOARD mode and arming...");
        Controller_Output_t init_output;
        init_output.thrust = 0.6;
        send_cmd(init_output, true); // send a zero command to initialize the offboard mode
        ++offboard_warmup_counter_;
        const ros::Time now = ros::Time::now();
        TrySetOffboard(now);
        TryArm(now);
        if(currState_.mode == "OFFBOARD" && currState_.armed){
            if(auto_takeoff_){
                ROS_INFO("Offboard and armed! Taking off...");
                desired_state_.p(0) = 0.0;
                desired_state_.p(1) = 0.0;
                desired_state_.p(2) = takeoff_height_;
                desired_state_.yaw = 0.0;
                flightState_ = TAKEOFF;
            }else{
                flightState_ = MISSION_EXECUTION;
            }
        }
        break;
    }
    case TAKEOFF:{
        ROS_INFO_ONCE("Auto Taking off...");
        Controller_Output_t output;
        if(se3_hopf_.calControl(odom_data_, imu_data_, desired_state_, output)){
            send_cmd(output, true);
            se3_hopf_.estimateTa(imu_data_.a);
        }
        if(fabs(odom_data_.p(2) - takeoff_height_) < 0.02){
            ROS_INFO("TakeOff Complete");
            flightState_ = MISSION_EXECUTION;
        }
        break;
    }
        
    case MISSION_EXECUTION:{
        ROS_INFO_ONCE("Mission execution...");
        Controller_Output_t output;
        if(se3_hopf_.calControl(odom_data_, imu_data_, desired_state_, output)){
            send_cmd(output, true);
            se3_hopf_.estimateTa(imu_data_.a);
        }
        break;
    }
    case LANDING: {
        landing_locked_ = true;
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
            flightState_ = LANDED;
            ROS_INFO("land enabled");
        }
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
    case EMERGENCY:
         ROS_ERROR("Emergency state! Please check the system.");
         flightState_ = LANDING;
         break;
    default:
        break;
    }
}

void Se3HopfCtrl::send_cmd(const Controller_Output_t &output, bool angle){
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
    // ROS_INFO_STREAM_THROTTLE(3.0, "Publishing command: bodyrates [" << cmd.body_rate.x << ", " << cmd.body_rate.y << ", " << cmd.body_rate.z << "], "
    //                          << "orientation [" << cmd.orientation.w << ", " << cmd.orientation.x << ", " << cmd.orientation.y << ", " << cmd.orientation.z << "], "
    //                          << "thrust: " << cmd.thrust );
    if(angle){
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE + 
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE + 
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    }else{
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    }
    cmd_pub_.publish(cmd);
}

void Se3HopfCtrl::pubLocalPose(const Eigen::Vector3d &pose) 
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];

    local_pos_pub_.publish(msg);
}

bool Se3HopfCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    ROS_INFO("trigger land!");
    flightState_ = LANDING;
    return true;
}

void Se3HopfCtrl::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_data_.feed(msg, enu_frame_, vel_in_body_);
    bool judge_x = ((odom_data_.p(0) >= geo_fence_[0]) || (odom_data_.p(0) <= -geo_fence_[0]));
    bool judge_y = ((odom_data_.p(1) >= geo_fence_[1]) || (odom_data_.p(1) <= -geo_fence_[1]));
    bool judge_z = (odom_data_.p(2) >= geo_fence_[2]);
    bool judge = (judge_x || judge_y || judge_z);
    if(judge && currState_.mode != mavros_msgs::State::MODE_PX4_LAND){
        flightState_ = EMERGENCY;
        // mavros_msgs::SetMode land_set_mode;
        // land_set_mode.request.custom_mode = mavros_msgs::State::MODE_PX4_LAND;
        // if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
        //     flightState_ = LANDED;
        //     ROS_WARN("obs Land enabled");
        // }
    }
}

void Se3HopfCtrl::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
    imu_data_.feed(msg, enu_frame_);
}

void Se3HopfCtrl::StateCallback(const mavros_msgs::State::ConstPtr &msg){
    currState_ = *msg;
    if (currState_.mode == "AUTO.LAND" && !landing_locked_) {
        landing_locked_ = true;
        ROS_WARN("se3_hopf landing lock enabled (AUTO.LAND detected).");
    }
}

void Se3HopfCtrl::TrySetOffboard(const ros::Time &now) {
    if (landing_locked_ || !enable_auto_offboard_) {
        return;
    }
    if (currState_.mode == "OFFBOARD") {
        return;
    }
    if (offboard_warmup_counter_ < offboard_warmup_count_) {
        return;
    }
    if ((now - last_mode_request_).toSec() < request_interval_) {
        return;
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        offboard_triggered_ = true;
        ROS_INFO_THROTTLE(2.0, "se3_hopf requested OFFBOARD mode.");
    } else {
        ROS_WARN_THROTTLE(2.0, "se3_hopf failed to request OFFBOARD mode.");
    }
    last_mode_request_ = now;
}

void Se3HopfCtrl::TryArm(const ros::Time &now) {
    if (landing_locked_ || !enable_auto_arm_) {
        return;
    }
    if (currState_.armed) {
        return;
    }
    if (enable_auto_offboard_ && currState_.mode != "OFFBOARD") {
        return;
    }
    if ((now - last_arm_request_).toSec() < request_interval_) {
        return;
    }

    arm_cmd.request.value = true;
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        arm_triggered_ = true;
        ROS_INFO_THROTTLE(2.0, "se3_hopf requested arming.");
    } else {
        ROS_WARN_THROTTLE(2.0, "se3_hopf failed to arm.");
    }
    last_arm_request_ = now;
}

void Se3HopfCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg) 
{
    if (msg->points.empty()) {
        ROS_WARN("Received empty trajectory message");
        return;
    }
    // command/trajectory
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg->points[0];

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
