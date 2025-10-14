/**
 * @author tfly
 */

#include "pid_controller/pid_control.h"

using namespace std;

pidCtrl::pidCtrl(const ros::NodeHandle &nh):nh_(nh){

    state_sub_ = nh_.subscribe("/mavros/state",10,&pidCtrl::state_cb,this);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &pidCtrl::pos_cb, this);
    vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &pidCtrl::vel_cb, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 10, &pidCtrl::imuCallback, this);
    simpleWaypoint_sub_ = nh_.subscribe<nav_msgs::Path>("/waypoint_generator/waypoints", 10, &pidCtrl::simpleWaypoint_cb, this);
    multiDOFJoint_sub_ = nh_.subscribe("/command/trajectory", 10, &pidCtrl::multiDOFJointCallback, this);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    setpoint_raw_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    land_service_ = nh_.advertiseService("/land", &pidCtrl::landCallback, this);

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &pidCtrl::controlLoop,this);

    arm_triggered_ = false;
    offboard_triggered_ = false;
    
    int type;
    nh_.param<bool>("enable_sim", sim_enable_, false);
    nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
    nh_.param<int>("pid_type", type, 0);
    nh_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
    nh_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
    nh_.param<double>("geo_fence/z", geo_fence_[2], 4.0);

    node_state_ = WAITING_FOR_CONNECTED;
    pid_type_ = static_cast<ControlType>(type);
    targetVel_ << 0.0, 0.0, 0.0;
    targetPos_ << 0, 0, takeoff_height_;
    
    yaw_ref_ = 0.0;
    init_pose_ << 0, 0, 0.5;
}

void pidCtrl::controlLoop(const ros::TimerEvent &event)
{
    double dt = event.current_real.toSec() - event.last_real.toSec();
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
            node_state_ = TAKEOFF;
            // last_ = ros::Time::now();
        }
        break;
    }
    case TAKEOFF:{
        computeTarget(dt);
        if(is_arrive(currPose_, targetPos_)){
            ROS_INFO("TakeOff Complete");
            node_state_ = MISSION_EXECUTION;
        }
        break;
    }
        
    case MISSION_EXECUTION:{
        computeTarget(dt);
        break;
    }
    case LANDING: {
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
            ROS_INFO("land enabled");
        }
        node_state_ = LANDED;
        break;
      }
    case LANDED:
        if(!currState_.armed){
            ROS_INFO("Landed. Please set to position control and disarm.");
            cmdloop_timer_.stop();
        }
        break;
    case EMERGENCY:
        pubLocalPose(Eigen::Vector3d(0.0, 0.0, 2.0));
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
}

void pidCtrl::trigger_offboard()
{
    if (sim_enable_) {
        // Enable OFFBoard mode and arm automatically
        // This will only run if the vehicle is simulated
        
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (currState_.mode != "OFFBOARD" && !offboard_triggered_) {

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

void pidCtrl::trigger_arm()
{
    // mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if( currState_.mode == "OFFBOARD"){
        if( !currState_.armed && !arm_triggered_){
            if( arming_client_.call(arm_cmd) &&arm_cmd.response.success){
                arm_triggered_ = true;
                ROS_INFO("Vehicle armed");
            }
        }
    }
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

    local_pos_pub_.publish(msg);
}

void pidCtrl::pubAttitudeTarget(const Eigen::Vector4d &target_attitude, const double thrust_des) {
    // cout << "check: "<<thrust_des<<endl;
    mavros_msgs::AttitudeTarget msg;
  
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.type_mask = 0b00000111;
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = std::max(0.0, std::min(1.0, thrust_des));
  
    setpoint_raw_attitude_pub_.publish(msg);
}

bool pidCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    ROS_INFO("trigger land!");
    node_state_ = LANDING;
    return true;
}

void pidCtrl::simpleWaypoint_cb(const nav_msgs::Path::ConstPtr& msg){
    waypoints_.clear();  // 清空之前的航点
    for(int i=0; i<msg->poses.size(); i++){
        waypoints_.push_back(msg->poses[i]);
    }
    ROS_INFO("Received %zu waypoints.", waypoints_.size());
}

void pidCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) 
{
    // command/trajectory
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  
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
}
void pidCtrl::pos_cb(const geometry_msgs::PoseStamped &msg)
{
    currPose_ << msg.pose.position.x,
                 msg.pose.position.y,
                 msg.pose.position.z;

    for(int i = 0;i<3;i++){
        if(currPose_[i] > geo_fence_[i]){
            node_state_ = EMERGENCY;
            break;
        }
    }
}
void pidCtrl::vel_cb(const geometry_msgs::TwistStamped &msg)
{
    currVel_ << msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z;
}
void pidCtrl::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    currAcc_ << msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z;
}

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