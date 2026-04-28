#include "lqr_controller/lqr_controller.hpp"
#include <dynamic_reconfigure/server.h>

namespace lqr {

LQR_Controller::LQR_Controller(ros::NodeHandle& nh)
    : nodeHandle_(nh)
    , lqr_quaternion_(nh)
    , lqr_euler_(nh)
    , flightState_(FlightState::WAITING_FOR_CONNECTED)
    , prevFlightState_(FlightState::WAITING_FOR_CONNECTED)
    , armTriggered_(false)
    , offboardTriggered_(false)
    , takeoffComplete_(false)
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

    int controlTypeInt = 0;
    nodeHandle_.param<int>("lqr_control_type", controlTypeInt, 0);
    controlType_ = (controlTypeInt == 0) ? ControlType::QUATERNION : ControlType::EULER;

    // Initialize subscribers
    stateSub_ = nodeHandle_.subscribe("/mavros/state", 10, &LQR_Controller::stateCallback, this);
    odomSub_ = nodeHandle_.subscribe("/mavros/local_position/odom", 1, &LQR_Controller::odomCallback, this);
    trajectorySub_ = nodeHandle_.subscribe("/command/trajectory", 1, &LQR_Controller::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    attitudePub_ = nodeHandle_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    localPosPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // Initialize service clients
    armingClient_ = nodeHandle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    setModeClient_ = nodeHandle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Initialize service servers
    landService_ = nodeHandle_.advertiseService("/land", &LQR_Controller::landCallback, this);

    // Initialize control timer
    controlTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &LQR_Controller::controlLoop, this);

    // Initialize target position
    targetPos_ << 0, 0, takeoffHeight_;

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
    if (flightState_ != prevFlightState_) {
        ROS_WARN_STREAM("LQR State changed: " << state2string(prevFlightState_) << " -> " << state2string(flightState_));
        prevFlightState_ = flightState_;
    }

    switch (flightState_)
    {
    case FlightState::WAITING_FOR_CONNECTED:
        // Wait for connection
        break;

    case FlightState::WAITING_FOR_OFFBOARD:
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = initPose_[0];
        pose.pose.position.y = initPose_[1];
        pose.pose.position.z = initPose_[2];
        localPosPub_.publish(pose);

        triggerOffboard();
        triggerArm();

        if (currentState_.mode == "OFFBOARD" && currentState_.armed) {
            targetPos_ << initPose_[0], initPose_[1], takeoffHeight_;
            ROS_INFO("Ready to take off. targetPos set to [%.2f, %.2f, %.2f], takeoffHeight=%.2f",
                     targetPos_[0], targetPos_[1], targetPos_[2], takeoffHeight_);
            flightState_ = FlightState::MISSION_EXECUTION;
        }
        break;
    }

    case FlightState::MISSION_EXECUTION:
    {
        // Use LQR for everything: takeoff, hover, trajectory tracking
        // Set hover reference to target position when no trajectory is available
        if(controlType_ == ControlType::QUATERNION) {
            lqr_quaternion_.setHoverReference(targetPos_[0], targetPos_[1], targetPos_[2]);
        } else {
            lqr_euler_.setHoverReference(targetPos_[0], targetPos_[1], targetPos_[2]);
        }

        // Check if takeoff is complete
        if (!takeoffComplete_ && isAtPosition(targetPos_, 0.3)) {
            takeoffComplete_ = true;
            ROS_INFO("Takeoff complete! Current pos [%.2f, %.2f, %.2f], target [%.2f, %.2f, %.2f]",
                     currentPos_[0], currentPos_[1], currentPos_[2],
                     targetPos_[0], targetPos_[1], targetPos_[2]);
        }

        // LQR control is handled via the odom callback in lqr_quaternion/lqr_euler
        // The controllers compute control outputs based on trajectory
        publishAttitude();
        break;
    }

    case FlightState::LANDING:
    {
        mavros_msgs::SetMode landMode;
        landMode.request.custom_mode = "AUTO.LAND";
        if (setModeClient_.call(landMode) && landMode.response.mode_sent) {
            ROS_INFO("Landing mode enabled");
        }
        flightState_ = FlightState::LANDED;
        break;
    }

    case FlightState::LANDED:
        if (!currentState_.armed) {
            ROS_INFO("Landed. Please switch to position control to disarm.");
            controlTimer_.stop();
        }
        break;

    case FlightState::EMERGENCY:
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2.0;
        localPosPub_.publish(pose);
        ROS_WARN_STREAM_THROTTLE(2.0, "EMERGENCY: Geofence violation! Holding position.");
        break;
    }
    }
}

void LQR_Controller::publishAttitude()
{
    mavros_msgs::AttitudeTarget bodyrateMsg;

    // Get LQR output based on control type
    Eigen::Matrix<double, 4, 1> output;

    if (controlType_ == ControlType::QUATERNION) {
        auto traj_control = lqr_quaternion_.getTrajectoryControl();
        auto gain = lqr_quaternion_.getGain();
        auto error = lqr_quaternion_.getError();
        auto ref = lqr_quaternion_.getRefStates();
        output = traj_control - gain * error;

        // Clamp outputs
        for (int i = 0; i < 3; i++) {
            if (output(i) > 2.0) output(i) = 2.0;
            else if (output(i) < -2.0) output(i) = -2.0;
        }

        // Debug output for takeoff/Mission execution
        // ROS_INFO_THROTTLE(2.0, "LQR Quaternion: pos [%.2f, %.2f, %.2f] error [%.2f, %.2f, %.2f]",
        //                   currentPos_(0), currentPos_(1), currentPos_(2),
        //                   error(0), error(1), error(2));

        double thrust_raw = output(3);

        // Compute thrust
        double thrust_n = output(3);
        double normalized_thrust = (hoverThrust_ * thrust_n) / gravity_;
        normalized_thrust = std::max(0.1, std::min(0.9, normalized_thrust));

        // ROS_INFO_THROTTLE(2.0,
        //           "LQR Quat ctrl: z=%.2f z_ref=%.2f ez=%.2f evz=%.2f uref4=%.2f raw4=%.2f norm=%.3f Kt[z,vz]=[%.3f, %.3f]",
        //           currentPos_(2), ref(2), error(2), error(9),
        //           traj_control(3), thrust_raw, normalized_thrust,
        //           gain(3, 2), gain(3, 9));

        // Publish body rate command
        bodyrateMsg.header.stamp = ros::Time::now();
        bodyrateMsg.header.frame_id = "map";
        bodyrateMsg.body_rate.x = output(0);
        bodyrateMsg.body_rate.y = output(1);
        bodyrateMsg.body_rate.z = output(2);
        bodyrateMsg.thrust = normalized_thrust;
        bodyrateMsg.type_mask = 128;  // Use body rates

        attitudePub_.publish(bodyrateMsg);
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

        // ROS_INFO_THROTTLE(2.0, "LQR Euler: pos [%.2f, %.2f, %.2f] error [%.2f, %.2f, %.2f]",
        //                   currentPos_(0), currentPos_(1), currentPos_(2),
        //                   error(0), error(1), error(2));

        // ROS_INFO_THROTTLE(2.0,
        //           "LQR Euler ctrl: z=%.2f z_ref=%.2f ez=%.2f evz=%.2f uref4=%.2f raw4=%.2f Kt[z,vz]=[%.3f, %.3f]",
        //           currentPos_(2), ref(2), error(2), error(8),
        //           traj_control(3), output(3),
        //           gain(3, 2), gain(3, 8));
        
        double thrust_n = output(3);
        double normalized_thrust = (hoverThrust_ * thrust_n) / gravity_;
        normalized_thrust = std::max(0.1, std::min(0.9, normalized_thrust));
        // Publish body rate command
        bodyrateMsg.header.stamp = ros::Time::now();
        bodyrateMsg.header.frame_id = "map";
        bodyrateMsg.body_rate.x = cmd_body_rate_baselink(0);
        bodyrateMsg.body_rate.y = cmd_body_rate_baselink(1);
        bodyrateMsg.body_rate.z = cmd_body_rate_baselink(2);
        bodyrateMsg.thrust = normalized_thrust;  // Use normalized thrust
        bodyrateMsg.type_mask = 128;  // Use body rates

        // ROS_INFO_THROTTLE(2.0, "LQR Euler cmd: rates_raw=[%.2f, %.2f, %.2f] rates=[%.2f, %.2f, %.2f] thrust_raw=%.2f thrust_norm=%.3f",
        //           output(0), output(1), output(2),
        //           cmd_body_rate_baselink(0), cmd_body_rate_baselink(1), cmd_body_rate_baselink(2), 
        //           output(3), normalized_thrust);

        attitudePub_.publish(bodyrateMsg);
    }
}

void LQR_Controller::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    currentState_ = *msg;

    if (flightState_ == FlightState::WAITING_FOR_CONNECTED && currentState_.connected) {
        ROS_INFO("Mavros connected");
        flightState_ = FlightState::WAITING_FOR_OFFBOARD;
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
            ROS_WARN("Geofence violation on axis %d: pos=[%.2f, %.2f, %.2f] limit=[%.2f, %.2f, %.2f]",
                     i, currentPos_[0], currentPos_[1], currentPos_[2],
                     geoFence_[0], geoFence_[1], geoFence_[2]);
            flightState_ = FlightState::EMERGENCY;
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

void LQR_Controller::trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
    // Pass trajectory to LQR controllers
    if(controlType_ == ControlType::QUATERNION) {
        lqr_quaternion_.setTrajectory(msg);
    } else {
        lqr_euler_.setTrajectory(msg);
    }
    // ROS_DEBUG("LQR Controller: Received trajectory with %zu points", msg.points.size());
}

void LQR_Controller::triggerOffboard()
{
    if (simEnable_) {
        mavros_msgs::SetMode offbMode;
        offbMode.request.custom_mode = "OFFBOARD";
        if (currentState_.mode != "OFFBOARD" && !offboardTriggered_) {
            if (setModeClient_.call(offbMode) && offbMode.response.mode_sent) {
                offboardTriggered_ = true;
                ROS_INFO("OFFBOARD mode enabled");
            }
        }
    } else {
        if (currentState_.mode != "OFFBOARD") {
            ROS_WARN("Not in simulation - please switch to OFFBOARD mode manually");
        }
    }
}

void LQR_Controller::triggerArm()
{
    mavros_msgs::CommandBool armCmd;
    armCmd.request.value = true;

    if (currentState_.mode == "OFFBOARD" && !currentState_.armed && !armTriggered_) {
        if (armingClient_.call(armCmd) && armCmd.response.success) {
            armTriggered_ = true;
            ROS_INFO("Vehicle armed");
        }
    }
}

bool LQR_Controller::landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    ROS_INFO("Land service called");
    flightState_ = FlightState::LANDING;
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
        case FlightState::WAITING_FOR_CONNECTED: return "WAITING_FOR_CONNECTED";
        case FlightState::WAITING_FOR_OFFBOARD: return "WAITING_FOR_OFFBOARD";
        case FlightState::TAKEOFF: return "TAKEOFF";
        case FlightState::MISSION_EXECUTION: return "MISSION_EXECUTION";
        case FlightState::LANDING: return "LANDING";
        case FlightState::LANDED: return "LANDED";
        case FlightState::EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

}  // namespace lqr