#include "lqr_controller/lqr_controller.hpp"
#include <dynamic_reconfigure/server.h>

namespace lqr {

LQR_Controller::LQR_Controller(ros::NodeHandle& nh)
    : nodeHandle_(nh)
    , lqr_quaternion_(nh)
    , lqr_euler_(nh)
    , flightState_(WAITING_FOR_CONNECTED)
    , prevFlightState_(WAITING_FOR_CONNECTED)
    , offboardWarmupCounter_(0)
    , offboardWarmupCount_(80)
    , requestInterval_(1.0)
{
    // Load parameters
    nodeHandle_.param<bool>("enable_sim", simEnable_, false);
    nodeHandle_.param<double>("takeoff_height", takeoffHeight_, 2.0);
    nodeHandle_.param<double>("mass", mass_, 1.5);
    nodeHandle_.param<double>("hover_thrust", hoverThrust_, 0.7);
    nodeHandle_.param<double>("init_pose_x", initPose_[0], 0.0);
    nodeHandle_.param<double>("init_pose_y", initPose_[1], 0.0);
    nodeHandle_.param<double>("init_pose_z", initPose_[2], 0.5);
    nodeHandle_.param<double>("geo_fence/x", geoFence_[0], 10.0);
    nodeHandle_.param<double>("geo_fence/y", geoFence_[1], 10.0);
    nodeHandle_.param<double>("geo_fence/z", geoFence_[2], 4.0);
    nodeHandle_.param<bool>("enable_auto_offboard", enableAutoOffboard_, simEnable_);
    nodeHandle_.param<bool>("enable_auto_arm", enableAutoArm_, simEnable_);
    nodeHandle_.param<bool>("auto_takeoff", autoTakeoff_, true);
    nodeHandle_.param<int>("offboard_warmup_count", offboardWarmupCount_, 80);
    nodeHandle_.param<double>("request_interval", requestInterval_, 1.0);

    int controlTypeInt = 0;
    nodeHandle_.param<int>("lqr_control_type", controlTypeInt, 0);
    controlType_ = (controlTypeInt == 0) ? ControlType::QUATERNION : ControlType::EULER;

    // Initialize subscribers
    stateSub_ = nodeHandle_.subscribe<mavros_msgs::State>("/mavros/state", 10, &LQR_Controller::stateCallback, this);
    odomSub_ = nodeHandle_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &LQR_Controller::odomCallback, this);
    trajectorySub_ = nodeHandle_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 1, &LQR_Controller::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    attitudePub_ = nodeHandle_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    localPosPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    flightStatePub_ = nodeHandle_.advertise<std_msgs::Int8>("/flight_state", 10);
    referencePosePub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/controller/reference_pose", 10);
    referenceVelPub_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("/controller/reference_velocity", 10);
    referenceAccPub_ = nodeHandle_.advertise<geometry_msgs::AccelStamped>("/controller/reference_accel", 10);

    // Initialize service clients
    armingClient_ = nodeHandle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    setModeClient_ = nodeHandle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Initialize service servers
    landService_ = nodeHandle_.advertiseService("/land", &LQR_Controller::landCallback, this);

    // Initialize control timer
    controlTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &LQR_Controller::controlLoop, this);

    // Initialize target position
    // targetPos_ << 0, 0, takeoffHeight_;
    if (offboardWarmupCount_ < 1) {
        offboardWarmupCount_ = 1;
    }
    if (requestInterval_ < 0.1) {
        requestInterval_ = 0.1;
    }
    lastModeRequest_ = ros::Time(0);
    lastArmRequest_ = ros::Time(0);

    // Initialize dynamic reconfigure server
    dynamic_reconfigure::Server<lqr_controller::LqrControllerConfig> srv;
    dynamic_reconfigure::Server<lqr_controller::LqrControllerConfig>::CallbackType f;
    f = boost::bind(&LQR_Controller::dynamicReconfigureCallback, this, _1, _2);
    srv.setCallback(f);

    ROS_INFO("LQR Controller initialized with control type: %s",
             controlType_ == ControlType::QUATERNION ? "QUATERNION" : "EULER");
}

LQR_Controller::~LQR_Controller()
{
}

void LQR_Controller::controlLoop(const ros::TimerEvent& event)
{
    std_msgs::Int8 flight_state_msg;
    flight_state_msg.data = static_cast<int8_t>(flightState_);
    flightStatePub_.publish(flight_state_msg);

    if (flightState_ != prevFlightState_) {
        ROS_WARN_STREAM("LQR State changed: " << state2string(prevFlightState_) << " -> " << state2string(flightState_));
        prevFlightState_ = flightState_;
    }

    switch (flightState_)
    {
    case WAITING_FOR_CONNECTED:
        ROS_INFO_ONCE("Waiting for FCU connection...");
        // Wait for connection
        if (currentState_.connected) {
            ROS_INFO("Connected to FCU!");
            offboardWarmupCounter_ = 0;
            flightState_ = WAITING_FOR_OFFBOARD;
        }
        break;

    case WAITING_FOR_OFFBOARD:
    {
        // geometry_msgs::PoseStamped pose;
        // pose.pose.position.x = initPose_[0];
        // pose.pose.position.y = initPose_[1];
        // pose.pose.position.z = initPose_[2];
        // localPosPub_.publish(pose);
        ROS_INFO_ONCE("Waiting for OFFBOARD mode and arming...");
        publishAttitude(Eigen::Vector4d(0, 0, 0, hoverThrust_));  // Publish hover command to help transition to OFFBOARD

        ++offboardWarmupCounter_;
        TrySetOffboard(ros::Time::now());
        TryArm(ros::Time::now());

        if (currentState_.mode == "OFFBOARD" && currentState_.armed) {
            if(autoTakeoff_) {
                ROS_INFO("Auto takeoff enabled. Transitioning to TAKEOFF state.");
                targetPos_ << initPose_[0], initPose_[1], takeoffHeight_;
                if(controlType_ == ControlType::QUATERNION) {
                    lqr_quaternion_.setHoverReference(targetPos_[0], targetPos_[1], targetPos_[2]);
                } else {
                    lqr_euler_.setHoverReference(targetPos_[0], targetPos_[1], targetPos_[2]);
                }
                flightState_ = TAKEOFF;
            } else {
                flightState_ = MISSION_EXECUTION;
            }
        }
        break;
    }
    case TAKEOFF:
    {
        // Publish takeoff command (handled in odom callback via LQR)
        ROS_INFO_ONCE("Auto Taking off...");
        Eigen::Vector4d cmd_body_rate_thrust;
        computeControlCommands(cmd_body_rate_thrust);
        publishAttitude(cmd_body_rate_thrust);
        if (isAtPosition(targetPos_, 0.3)) {
            flightState_ = MISSION_EXECUTION;
            ROS_INFO("Takeoff complete! Current pos [%.2f, %.2f, %.2f], target [%.2f, %.2f, %.2f]",
                     currentPos_[0], currentPos_[1], currentPos_[2],
                     targetPos_[0], targetPos_[1], targetPos_[2]);
        }
        break;
    }

    case MISSION_EXECUTION:   
    {
        ROS_INFO_ONCE("Executing mission...");
        // LQR control is handled via the odom callback in lqr_quaternion/lqr_euler
        // The controllers compute control outputs based on trajectory
        Eigen::Vector4d cmd_body_rate_thrust;
        computeControlCommands(cmd_body_rate_thrust);
        publishAttitude(cmd_body_rate_thrust);
        break;
    }

    case LANDING:
    {
        landingLocked_ = true;
        mavros_msgs::SetMode landMode;
        landMode.request.custom_mode = "AUTO.LAND";
        if (setModeClient_.call(landMode) && landMode.response.mode_sent) {
            flightState_ = LANDED;
            ROS_INFO("Landing mode enabled");
        }
        break;
    }

    case LANDED:
        if (!currentState_.armed) {
            ROS_INFO("Landed. Please switch to position control to disarm.");
            controlTimer_.stop();
        }
        break;

    case EMERGENCY:
    {
        // geometry_msgs::PoseStamped pose;
        // pose.pose.position.x = 0;
        // pose.pose.position.y = 0;
        // pose.pose.position.z = 2.0;
        // localPosPub_.publish(pose);
        flightState_ = LANDING;
        ROS_WARN_STREAM_THROTTLE(2.0, "EMERGENCY: Geofence violation! Holding position.");
        break;
    }
    }

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
    if (controlType_ == ControlType::QUATERNION) {
        auto ref = lqr_quaternion_.getRefStates();
        ref_vel_msg.twist.linear.x = ref(7);
        ref_vel_msg.twist.linear.y = ref(8);
        ref_vel_msg.twist.linear.z = ref(9);
    } else {
        auto ref = lqr_euler_.getRefStates();
        ref_vel_msg.twist.linear.x = ref(6);
        ref_vel_msg.twist.linear.y = ref(7);
        ref_vel_msg.twist.linear.z = ref(8);
    }
    referenceVelPub_.publish(ref_vel_msg);

    geometry_msgs::AccelStamped ref_acc_msg;
    ref_acc_msg.header = ref_msg.header;
    ref_acc_msg.accel.linear.x = 0.0;
    ref_acc_msg.accel.linear.y = 0.0;
    ref_acc_msg.accel.linear.z = 0.0;
    referenceAccPub_.publish(ref_acc_msg);
}

void LQR_Controller::computeControlCommands(Eigen::Vector4d& bodyRatesThrustCmd)
{
    // This function can be used to compute control commands based on the current state and trajectory
    // For now, the control computation is done in the odom callback via the LQR controllers
     Eigen::Matrix<double, 4, 1> output;

    if (controlType_ == ControlType::QUATERNION) {
        auto traj_control = lqr_quaternion_.getTrajectoryControl();
        auto gain = lqr_quaternion_.getGain();
        auto error = lqr_quaternion_.getError();
        auto ref = lqr_quaternion_.getRefStates();
        output = traj_control - gain * error;
        bodyRatesThrustCmd << output(0), output(1), output(2), output(3);
        // Clamp outputs
        // for (int i = 0; i < 3; i++) {
        //     if (output(i) > 2.0) output(i) = 2.0;
        //     else if (output(i) < -2.0) output(i) = -2.0;
        // }

        // Debug output for takeoff/Mission execution
        // ROS_INFO_THROTTLE(2.0, "LQR Quaternion: pos [%.2f, %.2f, %.2f] error [%.2f, %.2f, %.2f]",
        //                   currentPos_(0), currentPos_(1), currentPos_(2),
        //                   error(0), error(1), error(2));

        // double thrust_raw = output(3);

        // Compute thrust
        // double thrust_n = output(3);
        // double normalized_thrust = (hoverThrust_ * thrust_n) / gravity_;
        // normalized_thrust = std::max(0.1, std::min(0.9, normalized_thrust));

        // ROS_INFO_THROTTLE(2.0,
        //           "LQR Quat ctrl: z=%.2f z_ref=%.2f ez=%.2f evz=%.2f uref4=%.2f raw4=%.2f norm=%.3f Kt[z,vz]=[%.3f, %.3f]",
        //           currentPos_(2), ref(2), error(2), error(9),
        //           traj_control(3), thrust_raw, normalized_thrust,
        //           gain(3, 2), gain(3, 9));
        
    } else {
        auto traj_control = lqr_euler_.getTrajectoryControl();
        auto gain = lqr_euler_.getGain();
        auto error = lqr_euler_.getError();
        auto ref = lqr_euler_.getRefStates();
        output = traj_control - gain * error;

        // Call setOutput for EULER to apply thrust negation and clamping
        lqr_euler_.setOutput(output);
        output = lqr_euler_.getOutput();

        Eigen::Vector3d cmd_body_rate_aircraft;
        Eigen::Vector3d cmd_body_rate_baselink;
        cmd_body_rate_aircraft << output(0),
                                    output(1),
                                    output(2);

        cmd_body_rate_baselink = mavros::ftf::transform_frame_aircraft_baselink<Eigen::Vector3d>(cmd_body_rate_aircraft);
        bodyRatesThrustCmd << cmd_body_rate_baselink(0), cmd_body_rate_baselink(1), cmd_body_rate_baselink(2), output(3);
        // ROS_INFO_THROTTLE(2.0, "LQR Euler: pos [%.2f, %.2f, %.2f] error [%.2f, %.2f, %.2f]",
        //                 currentPos_(0), currentPos_(1), currentPos_(2),
        //                 error(0), error(1), error(2));

        // ROS_INFO_THROTTLE(2.0,
        //         "LQR Euler ctrl: z=%.2f z_ref=%.2f ez=%.2f evz=%.2f uref4=%.2f raw4=%.2f Kt[z,vz]=[%.3f, %.3f]",
        //         currentPos_(2), ref(2), error(2), error(8),
        //         traj_control(3), output(3),
        //         gain(3, 2), gain(3, 8));
        
        // double thrust_n = output(3);
        // double normalized_thrust = (hoverThrust_ * thrust_n) / gravity_;
        // normalized_thrust = std::max(0.1, std::min(0.9, normalized_thrust));
        
        // ROS_INFO_THROTTLE(2.0, "LQR Euler cmd: rates_raw=[%.2f, %.2f, %.2f] rates=[%.2f, %.2f, %.2f] thrust_raw=%.2f thrust_norm=%.3f",
        //         output(0), output(1), output(2),
        //         cmd_body_rate_baselink(0), cmd_body_rate_baselink(1), cmd_body_rate_baselink(2), 
        //         output(3), normalized_thrust);
    }
}

void LQR_Controller::publishAttitude(Eigen::Vector4d bodyRatesThrustCmd)
{
    mavros_msgs::AttitudeTarget bodyrateMsg;

    // Get LQR output based on control type
   

    // Clamp outputs
    // for (int i = 0; i < 3; i++) {
    //     if (output(i) > 2.0) output(i) = 2.0;
    //     else if (output(i) < -2.0) output(i) = -2.0;
    // }

    double thrust_raw = bodyRatesThrustCmd(3);

    // Compute thrust
    double normalized_thrust = (hoverThrust_ * thrust_raw) / gravity_;
    normalized_thrust = std::max(0.1, std::min(0.9, normalized_thrust));

    // Publish body rate command
    bodyrateMsg.header.stamp = ros::Time::now();
    bodyrateMsg.header.frame_id = "map";
    bodyrateMsg.body_rate.x = bodyRatesThrustCmd(0);
    bodyrateMsg.body_rate.y = bodyRatesThrustCmd(1);
    bodyrateMsg.body_rate.z = bodyRatesThrustCmd(2);
    bodyrateMsg.thrust = normalized_thrust;
    bodyrateMsg.type_mask = 128;  // Use body rates

    attitudePub_.publish(bodyrateMsg);
}

void LQR_Controller::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState_ = *msg;

    if (currentState_.mode == "AUTO.LAND" && !landingLocked_) {
        landingLocked_ = true;
        ROS_WARN("lqr_controller landing lock enabled (AUTO.LAND detected).");
    }
}

void LQR_Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentPos_ << msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z;

    // Check geofence
    for (int i = 0; i < 3; i++) {
        if (std::abs(currentPos_[i]) > geoFence_[i]) {
            if(flightState_ != FlightState::LANDING && flightState_ != FlightState::LANDED) {
                ROS_ERROR("Geofence violation! Current pos [%.2f, %.2f, %.2f] exceeds limit [%.2f, %.2f, %.2f]",
                          currentPos_[0], currentPos_[1], currentPos_[2],
                          geoFence_[0], geoFence_[1], geoFence_[2]);
                flightState_ = EMERGENCY;
            }
            
            break;
        }
    }

    // Update LQR controllers with current state
    if(controlType_ == ControlType::QUATERNION) {
        lqr_quaternion_.setStates(msg);
        // Compute LQR gains (with internal timing check)
        lqr_quaternion_.computeLQR();
    } else {
        lqr_euler_.setStates(msg);
        lqr_euler_.computeLQR();
    }
}

void LQR_Controller::trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    // Pass trajectory to LQR controllers
    if(controlType_ == ControlType::QUATERNION) {
        lqr_quaternion_.setTrajectory(*msg);
    } else {
        lqr_euler_.setTrajectory(*msg);
    }
    // ROS_DEBUG("LQR Controller: Received trajectory with %zu points", msg->points.size());
}

void LQR_Controller::TrySetOffboard(const ros::Time& now)
{
    if (landingLocked_ || !enableAutoOffboard_) {
        return;
    }
    if (currentState_.mode == "OFFBOARD") {
        return;
    }
    if (offboardWarmupCounter_ < offboardWarmupCount_) {
        return;
    }
    if ((now - lastModeRequest_).toSec() < requestInterval_) {
        return;
    }

    mavros_msgs::SetMode setMode;
    setMode.request.custom_mode = "OFFBOARD";
    if (setModeClient_.call(setMode) && setMode.response.mode_sent) {
        ROS_INFO_THROTTLE(2.0, "lqr_controller requested OFFBOARD mode.");
    } else {
        ROS_WARN_THROTTLE(2.0, "lqr_controller failed to request OFFBOARD mode.");
    }
    lastModeRequest_ = now;
}

void LQR_Controller::TryArm(const ros::Time& now)
{
    if (landingLocked_ || !enableAutoArm_) {
        return;
    }
    if (currentState_.armed) {
        return;
    }
    if (enableAutoOffboard_ && currentState_.mode != "OFFBOARD") {
        return;
    }
    if ((now - lastArmRequest_).toSec() < requestInterval_) {
        return;
    }

    mavros_msgs::CommandBool armCmd;
    armCmd.request.value = true;
    if (armingClient_.call(armCmd) && armCmd.response.success) {
        ROS_INFO_THROTTLE(2.0, "lqr_controller requested arming.");
    } else {
        ROS_WARN_THROTTLE(2.0, "lqr_controller failed to arm.");
    }
    lastArmRequest_ = now;
}

bool LQR_Controller::landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    ROS_INFO("Land service called");
    flightState_ = LANDING;
    response.success = true;
    return true;
}

void LQR_Controller::dynamicReconfigureCallback(lqr_controller::LqrControllerConfig &config, uint32_t level)
{
    // Update Q matrix for quaternion controller
    state_matrix_quat_t Q_quat;
    Q_quat.setZero();
    Q_quat.diagonal() << config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
                         config.Q_orient_w, config.Q_orient_x, config.Q_orient_y, config.Q_orient_z,
                         config.Q_vel_x, config.Q_vel_y, config.Q_vel_z;
    lqr_quaternion_.setQ(Q_quat);

    // Update R matrix for quaternion controller
    control_matrix_quat_t R_quat;
    R_quat.setZero();
    R_quat.diagonal() << config.R_rate_x, config.R_rate_y, config.R_rate_z, config.R_thrust;
    lqr_quaternion_.setR(R_quat);

    // Update Q matrix for euler controller (9 states)
    state_matrix_t Q_euler;
    Q_euler.setZero();
    Q_euler.diagonal() << config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
                          config.Q_orient_w, config.Q_orient_x, config.Q_orient_y,  // roll, pitch, yaw
                          config.Q_vel_x, config.Q_vel_y, config.Q_vel_z;
    lqr_euler_.setQ(Q_euler);

    // Update R matrix for euler controller
    control_matrix_t R_euler;
    R_euler.setZero();
    R_euler.diagonal() << config.R_rate_x, config.R_rate_y, config.R_rate_z, config.R_thrust;
    lqr_euler_.setR(R_euler);

    // Update control type if changed
    controlType_ = (config.lqr_control_type == 0) ? ControlType::QUATERNION : ControlType::EULER;

    ROS_INFO("LQR Dynamic Reconfigure: Q_pos [%.1f, %.1f, %.1f], Q_vel [%.1f, %.1f, %.1f], R_rates [%.1f, %.1f, %.1f], R_thrust %.1f",
             config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
             config.Q_vel_x, config.Q_vel_y, config.Q_vel_z,
             config.R_rate_x, config.R_rate_y, config.R_rate_z, config.R_thrust);
}

bool LQR_Controller::isAtPosition(const Eigen::Vector3d& target, double threshold)
{
    return (currentPos_ - target).norm() < threshold;
}

std::string LQR_Controller::state2string(FlightState state)
{
    switch (state) {
        case WAITING_FOR_CONNECTED: return "WAITING_FOR_CONNECTED";
        case WAITING_FOR_OFFBOARD: return "WAITING_FOR_OFFBOARD";
        case TAKEOFF: return "TAKEOFF";
        case MISSION_EXECUTION: return "MISSION_EXECUTION";
        case LANDING: return "LANDING";
        case LANDED: return "LANDED";
        case EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

}  // namespace lqr