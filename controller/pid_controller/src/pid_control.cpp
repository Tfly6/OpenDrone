#include "pid_controller/pid_control.h"

using namespace std;

pidCtrl::pidCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh), dyn_config_server_(private_nh_) {

    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state",10,&pidCtrl::state_cb,this);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &pidCtrl::pos_cb, this);
    vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &pidCtrl::vel_cb, this);
    simpleWaypoint_sub_ = nh_.subscribe<nav_msgs::Path>("/waypoint_generator/waypoints", 10, &pidCtrl::simpleWaypoint_cb, this);
    multiDOFJoint_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 10, &pidCtrl::multiDOFJointCallback, this);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    setpoint_raw_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("/controller/reference_pose", 10);
    referenceVelPub_ = nh_.advertise<geometry_msgs::TwistStamped>("/controller/reference_velocity", 10);
    referenceAccPub_ = nh_.advertise<geometry_msgs::AccelStamped>("/controller/reference_accel", 10);
    flight_state_pub_ = nh_.advertise<std_msgs::Int8>("/flight_state", 10);

    land_service_ = nh_.advertiseService("/land", &pidCtrl::landCallback, this);

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &pidCtrl::controlLoop,this);
    
    private_nh_.param<bool>("enable_sim", sim_enable_, false);
    private_nh_.param<bool>("enable_auto_offboard", enable_auto_offboard_, sim_enable_);
    private_nh_.param<bool>("enable_auto_arm", enable_auto_arm_, sim_enable_);
    private_nh_.param<bool>("auto_takeoff", autoTakeoff_, true);
    private_nh_.param<int>("offboard_warmup_count", offboard_warmup_count_, 80);
    private_nh_.param<double>("request_interval", request_interval_, 1.0);
    private_nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
    private_nh_.param<int>("pid_type", pid_type_, CASCADE_PID);
    private_nh_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
    private_nh_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
    private_nh_.param<double>("geo_fence/z", geo_fence_[2], 4.0);

    // nh_.param<double>("mass", uavMass_, 1.0);
    // limit

    // nh_.param<double>("Kp_x", Kp_x_, 1.0);
    // nh_.param<double>("Kp_y", Kp_y_, 1.0);
    // nh_.param<double>("Kp_z", Kp_z_, 2.0);
    // nh_.param<double>("Ki_x", Ki_x_, 0.2);
    // nh_.param<double>("Ki_y", Ki_y_, 0.2);
    // nh_.param<double>("Ki_z", Ki_z_, 0.2);
    // nh_.param<double>("Kd_x", Kd_x_, 0.5);
    // nh_.param<double>("Kd_y", Kd_y_, 0.5);
    // nh_.param<double>("Kd_z", Kd_z_, 0.5);



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
    targetVel_.setZero();
    targetPos_.setZero();
    targetAtt_.setZero();
    // targetVel_ << 0.0, 0.0, 0.0;
    // targetPos_ << 0, 0, takeoff_height_;

    // kp_ << Kp_x_, Kp_y_, Kp_z_;
    // ki_ << Ki_x_, Ki_y_, Ki_z_;
    // kd_ << Kd_x_, Kd_y_, Kd_z_;
    // pre_error_ << 0.0, 0.0, 0.0;
    // integral_ << 0.0, 0.0, 0.0;

    
    yaw_ref_ = 0.0;
    init_pose_ << 0, 0, 0.5;

    bool use_dynamic_reconfigure = false;
    private_nh_.param("use_dynamic_reconfigure", use_dynamic_reconfigure, false);
    if (use_dynamic_reconfigure) {
        dyn_config_cb_type_ = boost::bind(&pidCtrl::dynamicReconfigureCallback, this, _1, _2);
        dyn_config_server_.setCallback(dyn_config_cb_type_);
        ROS_INFO("pid_controller using dynamic_reconfigure for tuning parameters.");
    } else {
        ROS_INFO("pid_controller using static ROS parameters for tuning parameters.");
    }
}

void pidCtrl::controlLoop(const ros::TimerEvent &event)
{
    std_msgs::Int8 flight_state_msg;
    flight_state_msg.data = static_cast<int8_t>(flightState_);
    flight_state_pub_.publish(flight_state_msg);

    geometry_msgs::PoseStamped ref_msg;
    ref_msg.header.stamp = ros::Time::now();
    ref_msg.header.frame_id = "map";
    ref_msg.pose.position.x = targetPos_(0);
    ref_msg.pose.position.y = targetPos_(1);
    ref_msg.pose.position.z = targetPos_(2);
    ref_msg.pose.orientation.w = 1.0;
    referencePosePub_.publish(ref_msg);

    geometry_msgs::TwistStamped ref_vel_msg;
    ref_vel_msg.header = ref_msg.header;
    ref_vel_msg.twist.linear.x = targetVel_(0);
    ref_vel_msg.twist.linear.y = targetVel_(1);
    ref_vel_msg.twist.linear.z = targetVel_(2);
    referenceVelPub_.publish(ref_vel_msg);

    geometry_msgs::AccelStamped ref_acc_msg;
    ref_acc_msg.header = ref_msg.header;
    ref_acc_msg.accel.linear.x = targetAcc_(0);
    ref_acc_msg.accel.linear.y = targetAcc_(1);
    ref_acc_msg.accel.linear.z = targetAcc_(2);
    referenceAccPub_.publish(ref_acc_msg);

    if (flightState_ != prev_flightState_) {
        ROS_WARN_STREAM("State changed from " << state2string(prev_flightState_) << " to " << state2string(flightState_));
        prev_flightState_ = flightState_;
    }
    double dt = event.current_real.toSec() - event.last_real.toSec();
    switch (flightState_)
    {
    case WAITING_FOR_CONNECTED:{
        ROS_INFO_ONCE("Waiting for FCU connection...");
        if(currState_.connected){
            ROS_INFO("FCU Connected");
            offboard_warmup_counter_ = 0;
            flightState_ = WAITING_FOR_OFFBOARD;
        }
        break;
    }
    case WAITING_FOR_OFFBOARD:{
        ROS_INFO_ONCE("Waiting for OFFBOARD mode and arming...");
        pubLocalPose(init_pose_);
        ++offboard_warmup_counter_;
        const ros::Time now = ros::Time::now();
        TrySetOffboard(now);
        TryArm(now);
        if(currState_.mode == "OFFBOARD" && currState_.armed){
            if (autoTakeoff_) {
                targetPos_ << init_pose_[0], init_pose_[1], takeoff_height_;
                flightState_ = TAKEOFF;
            } else {
                flightState_ = MISSION_EXECUTION;
            }
            // last_ = ros::Time::now();
        }
        break;
    }
    case TAKEOFF:{
        ROS_INFO_ONCE("Auto Taking off...");
        computeTarget(dt);
        if(is_arrive(currPose_, targetPos_)){
            ROS_INFO("TakeOff Complete");
            flightState_ = MISSION_EXECUTION;
        }
        break;
    }
        
    case MISSION_EXECUTION:{
        ROS_INFO_ONCE("Executing mission...");
        computeTarget(dt);
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
        break;
      }
    case LANDED:
        if(!currState_.armed){
            ROS_INFO("Landed. Please set to position control and disarm.");
            cmdloop_timer_.stop();
        }
        break;
    case EMERGENCY:
        // pubLocalPose(Eigen::Vector3d(0.0, 0.0, 2.0));
        flightState_ = LANDING;
        ROS_WARN_STREAM_THROTTLE(2.0, "emergency! please switch to land");
        break;
        
    default:
        break;
    }
}

void pidCtrl::computeTarget(const double dt)
{
    if(pid_type_ == SIMPLE_PID){
            
        // simpleController.printf_param();
        Eigen::Vector3d pose = simpleController.compute(currPose_, targetPos_, dt);
        pubLocalPose(pose);
    }
    else if (pid_type_ == CASCADE_PID){
        // cascadeController.printf_param();
        targetAtt_ = cascadeController.compute(currPose_, currVel_, targetPos_, yaw_ref_, dt);
        pubAttitudeTarget(targetAtt_, cascadeController.getDesiredThrust());
    }
    else{
        ROS_ERROR_ONCE("Invalid PID type! Defaulting to CASCADE_PID.");
        pid_type_ = CASCADE_PID;
    }
}

void pidCtrl::TrySetOffboard(const ros::Time &now)
{
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
        ROS_INFO_THROTTLE(2.0, "pid_controller requested OFFBOARD mode.");
    } else {
        ROS_WARN_THROTTLE(2.0, "pid_controller failed to request OFFBOARD mode.");
    }
    last_mode_request_ = now;
}

void pidCtrl::TryArm(const ros::Time &now)
{
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
        ROS_INFO_THROTTLE(2.0, "pid_controller requested arming.");
    } else {
        ROS_WARN_THROTTLE(2.0, "pid_controller failed to arm.");
    }
    last_arm_request_ = now;
}

void pidCtrl::pubVel(const Eigen::Vector3d &vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = vel[0];
    msg.linear.y = vel[1];
    msg.linear.z = vel[2];

    vel_pub_.publish(msg);
}

void pidCtrl::pubLocalPose(const Eigen::Vector3d &pose) 
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];
    // 设置单位四元数，避免 MAVROS 提取 yaw 时产生 NaN
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;

    // 保护：确保发布的坐标不含 NaN/Inf
    if (std::isfinite(pose[0]) && std::isfinite(pose[1]) && std::isfinite(pose[2])) {
        local_pos_pub_.publish(msg);
    } else {
        ROS_WARN_THROTTLE(1.0, "pid_controller: NaN/Inf detected in position output, skipping publish");
    }
}

void pidCtrl::pubAttitudeTarget(const Eigen::Vector4d &target_attitude, const double thrust_des) {
    // cout << "check: "<<thrust_des<<endl;
    mavros_msgs::AttitudeTarget msg;
  
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE + 
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE + 
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = std::max(0.0, std::min(1.0, thrust_des));
  
    setpoint_raw_attitude_pub_.publish(msg);
}

bool pidCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    ROS_INFO("trigger land!");
    flightState_ = LANDING;
    return true;
}

void pidCtrl::simpleWaypoint_cb(const nav_msgs::Path::ConstPtr& msg){
    waypoints_.clear();  // 清空之前的航点
    for(int i=0; i<msg->poses.size(); i++){
        waypoints_.push_back(msg->poses[i]);
    }
    ROS_INFO("Received %zu waypoints.", waypoints_.size());
}

void pidCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg) 
{
    if (msg->points.empty()) {
        ROS_WARN("Received empty trajectory message");
        return;
    }
    // command/trajectory
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg->points[0];
    // reference_request_last_ = reference_request_now_;
  
    // reference_request_now_ = ros::Time::now();
    // reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();
  
    targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
    targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;
  
    targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;

    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
        pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    yaw_ref_ = rpy(2);
}

void pidCtrl::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    currState_ = *msg;
    if (currState_.mode == "AUTO.LAND" && !landing_locked_) {
        landing_locked_ = true;
        ROS_WARN("pid_controller landing lock enabled (AUTO.LAND detected).");
    }
}
void pidCtrl::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    currPose_ << msg->pose.position.x,
                 msg->pose.position.y,
                 msg->pose.position.z;

    for(int i = 0;i<3;i++){
        if(currPose_[i] > geo_fence_[i] || currPose_[i] < -geo_fence_[i]){
            if (flightState_ != LANDING && flightState_ != LANDED) 
                flightState_ = EMERGENCY;
            break;
        }
    }
}
void pidCtrl::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    currVel_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z;
}
// void pidCtrl::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
//     currAcc_ << msg->linear_acceleration.x,
//                 msg->linear_acceleration.y,
//                 msg->linear_acceleration.z;
// }

void pidCtrl::dynamicReconfigureCallback(pid_controller::PidControllerConfig &config, uint32_t level){
    Eigen::Vector3d kp_p, kp_v, ki_v, kd_v;
    double pos_error_max, vel_error_max, vel_integral_max;

    kp_p << config.kp_px, config.kp_py, config.kp_pz;
    kp_v << config.kp_vx, config.kp_vy, config.kp_vz;
    ki_v << config.ki_vx, config.ki_vy, config.ki_vz;
    kd_v << config.kd_vx, config.kd_vy, config.kd_vz;

    pos_error_max = config.pos_error_max;
    vel_error_max = config.vel_error_max;
    vel_integral_max = config.vel_integral_max;

    cascadeController.setParam(kp_p, kp_v, ki_v, kd_v,
                        pos_error_max, vel_error_max, vel_integral_max);
    cascadeController.printf_param();
}
