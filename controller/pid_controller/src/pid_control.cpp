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
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    setpoint_raw_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &pidCtrl::controlLoop,this);

    arm_triggered_ = false;
    offboard_triggered_ = false;
    
    // double error_max_x, error_max_x, error_max_x
    nh_.param<bool>("enable_sim", sim_enable_, false);
    nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
    // nh_.param<double>("mass", uavMass_, 1.0);
    // limit
    // nh_.param<double>("ff_gain", ff_gain_, 0.5);
    // nh_.param<double>("tilt_max", tilt_max_, 5.0);
    // nh_.param<double>("error_max_x", pos_error_max_[0], 0.6);
    // nh_.param<double>("error_max_y", pos_error_max_[1], 0.6);
    // nh_.param<double>("error_max_z", pos_error_max_[2], 0.5);

    // nh_.param<double>("Kp_x", Kp_x_, 1.0);
    // nh_.param<double>("Kp_y", Kp_y_, 1.0);
    // nh_.param<double>("Kp_z", Kp_z_, 2.0);
    // nh_.param<double>("Ki_x", Ki_x_, 0.2);
    // nh_.param<double>("Ki_y", Ki_y_, 0.2);
    // nh_.param<double>("Ki_z", Ki_z_, 0.2);
    // nh_.param<double>("Kd_x", Kd_x_, 0.5);
    // nh_.param<double>("Kd_y", Kd_y_, 0.5);
    // nh_.param<double>("Kd_z", Kd_z_, 0.5);



    node_state_ = WAITING_FOR_CONNECTED;
    pid_type_ = SIMPLE_PID;
    targetVel_ << 0.0, 0.0, 0.0;
    targetPos_ << 0, 0, takeoff_height_;

    // kp_ << Kp_x_, Kp_y_, Kp_z_;
    // ki_ << Ki_x_, Ki_y_, Ki_z_;
    // kd_ << Kd_x_, Kd_y_, Kd_z_;
    // pre_error_ << 0.0, 0.0, 0.0;
    // integral_ << 0.0, 0.0, 0.0;

    
    // yaw_ref_ = 0.0;
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
        
        //send a few setpoints before starting
        // for(int i = 100; ros::ok() && i > 0; --i){
        //     // cout << "check"<<endl;
        //     pubLocalPose(init_pose_);
        //     ros::spinOnce();
        // }
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
        // ros::spinOnce();
        break;
      }
    case LANDED:
    if(!currState_.armed){
        ROS_INFO("Landed. Please set to position control and disarm.");
        cmdloop_timer_.stop();
    }
    // ros::spinOnce();
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
            // cout << "check1"<< arming_client_.call(arm_cmd)<<endl;
        }
    }
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
    cout << "check: "<<thrust_des<<endl;
    mavros_msgs::AttitudeTarget msg;
  
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.type_mask = 0b00000111;  // Ignore orientation messages
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = std::max(0.0, std::min(1.0, thrust_des));
  
    setpoint_raw_attitude_pub_.publish(msg);
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
}
void pidCtrl::pos_cb(const geometry_msgs::PoseStamped &msg)
{
    currPose_[0] = msg.pose.position.x;
    currPose_[1] = msg.pose.position.y;
    currPose_[2] = msg.pose.position.z;
}
void pidCtrl::vel_cb(const geometry_msgs::TwistStamped &msg)
{
    currVel_[0] = msg.twist.linear.x;
    currVel_[1] = msg.twist.linear.y;
    currVel_[2] = msg.twist.linear.z;
}
void pidCtrl::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    currAcc_ << msg->linear_acceleration.x,
                   msg->linear_acceleration.y,
                   msg->linear_acceleration.z;
}

// void pidCtrl::compute(Eigen::Vector4d &targetAtt, double dt)
// {
//     Eigen::Vector3d acc_des;
    
//     // 计算误差项
//     Eigen::Vector3d pos_error = targetPos_ - currPose_;
//     // Eigen::Vector3f vel_error;

//     // 误差项限幅
//     for (int i=0; i<3; i++)
//     {
//         pos_error[i] = constrain_function(pos_error[i], pos_error_max_[i]);
//         // vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
//     }
//     integral_ += pos_error * dt;
    
//     Eigen::Vector3d derivative = pos_error - pre_error_;
//     derivative = derivative / dt;

//     // 期望加速度 = 加速度前馈 + PID
//     acc_des = currAcc_ + kp_.asDiagonal() * pos_error + ki_.asDiagonal() * integral_ + kd_.asDiagonal() * derivative;
    
//     // acc_des += gravity_;

//     // 转换为姿态和推力
//     // 水平方向控制量转换为姿态角
//     double pitch = acc_des[0] / gravity_;  // 前馈修正俯仰角
//     double roll = -acc_des[1] / gravity_;  // 注意坐标系方向

//     // 构造四元数
//     Eigen::Quaterniond q_des;
//     q_des = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *  // 保持yaw=0
//         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
//         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
//     targetAtt = Eigen::Vector4d(q_des.w(),q_des.x(),q_des.y(),q_des.z());
    
        
//     // 推力计算（考虑重力补偿）
//     targetThrust_ = ( acc_des[2] + gravity_) / gravity_;  // 归一化到0~1范围

//     // Eigen::Vector3d thrust_des;
//     // Eigen::Vector3d throttle_des;
//     // thrust_des =  accelToThrust(acc_des, uavMass_, tilt_max_);
//     // throttle_des = thrustToThrottle(thrust_des);
//     // targetAtt = ThrottleToAttitude(throttle_des, yaw_ref_);

    
//     // for (int i=0; i<3; i++)
//     // {
//     //     accel_sp[i] = _Reference_State.acceleration_ref[i] + kd_[i] * pos_error[i] + Kd[i] * vel_error[i] + Ki[i] * integral[i];
//     // }
    
//     // accel_sp[2] = accel_sp[2] + 9.8;

//     // 更新积分项
//     // for (int i=0; i<3; i++)
//     // {
//     //     if(abs(pos_error[i]) < int_start_error)
//     //     {
//     //         integral[i] += pos_error[i] * dt;

//     //         if(abs(integral[i]) > int_max[i])
//     //         {
//     //             cout << "Integral saturation! " << " [0-1-2] "<< i <<endl;
//     //             cout << "[integral]: "<< integral[i]<<" [int_max]: "<<int_max[i]<<" [m/s] "<<endl;
//     //         }

//     //         integral[i] = constrain_function(integral[i], int_max[i]);
//     //     }else
//     //     {
//     //         integral[i] = 0;
//     //     }

//     //     // If not in OFFBOARD mode, set all intergral to zero.
//     //     if(_DroneState.mode != "OFFBOARD")
//     //     {
//     //         integral[i] = 0;
//     //     }
//     // }

//     // 期望推力 = 期望加速度 × 质量
//     // 归一化推力 ： 根据电机模型，反解出归一化推力
//     // Eigen::Vector3d thrust_des;
//     // Eigen::Vector3d throttle_des;
//     // thrust_des =  accelToThrust(acc_des, uavMass_, tilt_max_);
//     // throttle_des = thrustToThrottle(thrust_des);

//     // for (int i=0; i<3; i++)
//     // {
//     //     _ControlOutput.Thrust[i] = thrust_sp[i];
//     //     _ControlOutput.Throttle[i] = throttle_sp[i];
//     // }

//     // return _ControlOutput;

// }

// Eigen::Vector3d pidCtrl::accelToThrust(const Eigen::Vector3d& accel_sp, double mass, double tilt_max)
// {
//     Eigen::Vector3d thrust_sp;

//     //除以电机个数得到单个电机的期望推力
//     thrust_sp = mass * accel_sp / NUM_MOTOR;

//     // 推力限幅，根据最大倾斜角及最大油门
//     double thrust_max_XY_tilt = fabs(thrust_sp[2]) * tanf(tilt_max/180.0*M_PI);
//     double thrust_max_XY = sqrtf(thrust_max_single_motor * thrust_max_single_motor - pow(thrust_sp[2],2));
//     thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

//     if ((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)) > pow(thrust_max_XY,2)) 
//     {
//         float mag = sqrtf((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)));
//         thrust_sp[0] = thrust_sp[0] / mag * thrust_max_XY;
//         thrust_sp[1] = thrust_sp[1] / mag * thrust_max_XY;
//     }
    
//     return thrust_sp;   
// }

// Eigen::Vector3d pidCtrl::thrustToThrottle(const Eigen::Vector3d& thrust_sp)
// {
//     Eigen::Vector3d throttle_sp;

//     //电机模型，可通过辨识得到，推力-油门曲线
//     for (int i=0; i<3; i++)
//     {
//         throttle_sp[i] = MOTOR_P1 * pow(thrust_sp[i],4) + MOTOR_P2 * pow(thrust_sp[i],3) + MOTOR_P3 * pow(thrust_sp[i],2) + MOTOR_P4 * thrust_sp[i] + MOTOR_P5;
//         // PX4内部默认假设 0.5油门为悬停推力 ， 在无人机重量为1kg时，直接除20得到0.5
//         // throttle_sp[i] = thrust_sp[i]/20；
//     }
//     return throttle_sp; 
// }

// //Throttle to Attitude
// //Thrust to Attitude
// //Input: desired thrust (desired throttle [0,1]) and yaw_sp(rad)
// //Output: desired attitude (quaternion)
// Eigen::Vector4d pidCtrl::ThrottleToAttitude(const Eigen::Vector3d& thr_sp, double yaw_sp)
// {
//     Eigen::Vector3d att_sp;
//     att_sp[2] = yaw_sp;

//     // desired body_z axis = -normalize(thrust_vector)
//     Eigen::Vector3d body_x, body_y, body_z;

//     double thr_sp_length = thr_sp.norm();
//     targetThrust_ = thr_sp_length;

//     //cout << "thr_sp_length : "<< thr_sp_length << endl;

//     if (thr_sp_length > 0.00001f) {
//             body_z = thr_sp.normalized();

//     } else {
//             // no thrust, set Z axis to safe value
//             body_z = Eigen::Vector3d(0.0f, 0.0f, 1.0f);
//     }

//     // vector of desired yaw direction in XY plane, rotated by PI/2
//     Eigen::Vector3d y_C = Eigen::Vector3d(-sin(yaw_sp),cos(yaw_sp),0.0);

//     if (fabsf(body_z(2)) > 0.000001f) {
//             // desired body_x axis, orthogonal to body_z
//             body_x = y_C.cross(body_z);

//             // keep nose to front while inverted upside down
//             if (body_z(2) < 0.0f) {
//                     body_x = -body_x;
//             }

//             body_x.normalize();

//     } else {
//             // desired thrust is in XY plane, set X downside to construct correct matrix,
//             // but yaw component will not be used actually
//             body_x = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
//             body_x(2) = 1.0f;
//     }

//     // desired body_y axis
//     body_y = body_z.cross(body_x);

//     Eigen::Matrix3d R_sp;

//     // fill rotation matrix
//     for (int i = 0; i < 3; i++) {
//             R_sp(i, 0) = body_x(i);
//             R_sp(i, 1) = body_y(i);
//             R_sp(i, 2) = body_z(i);
//     }

//     Eigen::Vector4d q_des = rot2Quaternion(R_sp);
//     return q_des;

//     //cout << "Desired euler [R P Y]: "<< att_sp[0]* 180/M_PI <<" [deg] " << att_sp[1]* 180/M_PI <<" [deg] "<< att_sp[2]* 180/M_PI <<" [deg] "<< endl;
//     //cout << "Desired Thrust: "<< thr_sp_length<< endl;
// //    cout << "q_sp [x y z w]: "<< q_sp.x() <<" [ ] " << q_sp.y() <<" [ ] "<<q_sp.z() <<" [ ] "<<q_sp.w() <<" [ ] "<<endl;
// //    cout << "R_sp : "<< R_sp(0, 0) <<" " << R_sp(0, 1) <<" "<< R_sp(0, 2) << endl;
// //    cout << "     : "<< R_sp(1, 0) <<" " << R_sp(1, 1) <<" "<< R_sp(1, 2) << endl;
// //    cout << "     : "<< R_sp(2, 0) <<" " << R_sp(2, 1) <<" "<< R_sp(2, 2) << endl;


//     // _AttitudeReference.throttle_sp[0] = thr_sp[0];
//     // _AttitudeReference.throttle_sp[1] = thr_sp[1];
//     // _AttitudeReference.throttle_sp[2] = thr_sp[2];

//     // //期望油门
//     // _AttitudeReference.desired_throttle = thr_sp_length; 

//     // _AttitudeReference.desired_att_q.w = q_sp.w();
//     // _AttitudeReference.desired_att_q.x = q_sp.x();
//     // _AttitudeReference.desired_att_q.y = q_sp.y();
//     // _AttitudeReference.desired_att_q.z = q_sp.z();

//     // _AttitudeReference.desired_attitude[0] = att_sp[0];  
//     // _AttitudeReference.desired_attitude[1] = att_sp[1]; 
//     // _AttitudeReference.desired_attitude[2] = att_sp[2]; 

//     // return _AttitudeReference;
// }

