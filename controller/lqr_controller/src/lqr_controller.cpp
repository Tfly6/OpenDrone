#include "lqr_controller/lqr_controller.hpp"
#include <dynamic_reconfigure/server.h>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace lqr {

LQR_Controller::LQR_Controller(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nodeHandle_(nh)
    , privateNodeHandle_(private_nh)
    , dynConfigServer_(privateNodeHandle_)
    , lqr_quaternion_(nh)
    , flightState_(WAITING_FOR_CONNECTED)
    , prevFlightState_(WAITING_FOR_CONNECTED)
    , offboardWarmupCounter_(0)
    , offboardWarmupCount_(80)
    , requestInterval_(1.0)
{
    // Load parameters
    privateNodeHandle_.param<bool>("enable_sim", simEnable_, false);
    privateNodeHandle_.param<double>("takeoff_height", takeoffHeight_, 2.0);
    privateNodeHandle_.param<double>("mass", mass_, 1.5);
    privateNodeHandle_.param<double>("hover_thrust", hoverThrust_, 0.7);
    privateNodeHandle_.param<double>("init_pose_x", initPose_[0], 0.0);
    privateNodeHandle_.param<double>("init_pose_y", initPose_[1], 0.0);
    privateNodeHandle_.param<double>("init_pose_z", initPose_[2], 0.5);
    privateNodeHandle_.param<double>("geo_fence/x", geoFence_[0], 10.0);
    privateNodeHandle_.param<double>("geo_fence/y", geoFence_[1], 10.0);
    privateNodeHandle_.param<double>("geo_fence/z", geoFence_[2], 4.0);
    privateNodeHandle_.param<bool>("enable_auto_offboard", enableAutoOffboard_, simEnable_);
    privateNodeHandle_.param<bool>("enable_auto_arm", enableAutoArm_, simEnable_);
    privateNodeHandle_.param<bool>("auto_takeoff", autoTakeoff_, true);
    privateNodeHandle_.param<bool>("use_dynamic_reconfigure", useDynamicReconfigure_, false);
    privateNodeHandle_.param<int>("offboard_warmup_count", offboardWarmupCount_, 80);
    privateNodeHandle_.param<double>("request_interval", requestInterval_, 1.0);
    privateNodeHandle_.param<double>("max_body_rate_xy", maxBodyRateXY_, 0.8);
    privateNodeHandle_.param<double>("max_body_rate_z", maxBodyRateZ_, 0.6);
    double max_tilt_deg = 30.0;
    privateNodeHandle_.param<double>("max_tilt_deg", max_tilt_deg, max_tilt_deg);
    privateNodeHandle_.param<double>("tilt_recovery_gain", tiltRecoveryGain_, 1.5);
    privateNodeHandle_.param<double>("min_normalized_thrust", minNormalizedThrust_, 0.15);
    privateNodeHandle_.param<double>("max_normalized_thrust", maxNormalizedThrust_, 0.85);

    // Initialize subscribers
    stateSub_ = nodeHandle_.subscribe<mavros_msgs::State>("/mavros/state", 10, &LQR_Controller::stateCallback, this);
    odomSub_ = nodeHandle_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &LQR_Controller::odomCallback, this);
    trajectorySub_ = nodeHandle_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 1, &LQR_Controller::trajectoryCallback, this, ros::TransportHints().tcpNoDelay());

    // Initialize publishers
    attitudePub_ = nodeHandle_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    localPosPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    flightStatePub_ = nodeHandle_.advertise<std_msgs::Int8>("/flight_state", 10);

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
    maxBodyRateXY_ = std::max(0.05, maxBodyRateXY_);
    maxBodyRateZ_ = std::max(0.05, maxBodyRateZ_);
    maxTiltRad_ = std::max(5.0, std::min(80.0, max_tilt_deg)) * M_PI / 180.0;
    tiltRecoveryGain_ = std::max(0.1, tiltRecoveryGain_);
    minNormalizedThrust_ = std::max(0.0, std::min(0.95, minNormalizedThrust_));
    maxNormalizedThrust_ = std::max(minNormalizedThrust_ + 0.01,
                                     std::min(1.0, maxNormalizedThrust_));
    lastModeRequest_ = ros::Time(0);
    lastArmRequest_ = ros::Time(0);
    targetPos_.setZero();

    if (!useDynamicReconfigure_) {
        loadStaticTuningConfig();
        ROS_INFO("LQR controller using static ROS parameters for tuning parameters.");
    } else {
        dynConfigCallbackType_ = boost::bind(&LQR_Controller::dynamicReconfigureCallback, this, _1, _2);
        dynConfigServer_.setCallback(dynConfigCallbackType_);
        ROS_INFO("LQR controller waiting for dynamic_reconfigure tuning parameters.");
    }

    ROS_INFO("LQR Controller initialized: max rates [%.2f, %.2f, %.2f] rad/s, max tilt %.1f deg, thrust [%.2f, %.2f]",
             maxBodyRateXY_, maxBodyRateXY_, maxBodyRateZ_,
             maxTiltRad_ * 180.0 / M_PI,
             minNormalizedThrust_, maxNormalizedThrust_);
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
        publishAttitude(Eigen::Vector4d(0, 0, 0, 9.81));  // Publish hover command to help transition to OFFBOARD

        ++offboardWarmupCounter_;
        TrySetOffboard(ros::Time::now());
        TryArm(ros::Time::now());

        if (currentState_.mode == "OFFBOARD" && currentState_.armed) {
            if(autoTakeoff_) {
                ROS_INFO("Auto takeoff enabled. Transitioning to TAKEOFF state.");
                targetPos_ << initPose_[0], initPose_[1], takeoffHeight_;
                lqr_quaternion_.setHoverReference(targetPos_[0], targetPos_[1], targetPos_[2]);
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
        if (isAtPosition(targetPos_, 0.15)) {
            flightState_ = MISSION_EXECUTION;
            missionEntryDebugLogged_ = false;
            ROS_INFO("Takeoff complete! Current pos [%.2f, %.2f, %.2f], target [%.2f, %.2f, %.2f]",
                     currentPos_[0], currentPos_[1], currentPos_[2],
                     targetPos_[0], targetPos_[1], targetPos_[2]);
        }
        break;
    }

    case MISSION_EXECUTION:   
    {
        ROS_INFO_ONCE("Executing mission...");
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

}

void LQR_Controller::computeControlCommands(Eigen::Vector4d& bodyRatesThrustCmd)
{
    // This function can be used to compute control commands based on the current state and trajectory
    // For now, the control computation is done in the odom callback via the LQR controllers
    Eigen::Matrix<double, 4, 1> output;

    auto traj_control = lqr_quaternion_.getTrajectoryControl();
    auto gain = lqr_quaternion_.getGain();
    auto error = lqr_quaternion_.getError();
    auto raw_error = error;
    // Saturate position error (ENU) to keep LQR within its linear operating range.
    // Without this, large initial errors (e.g. 5 m offset to circle start) produce
    // excessively aggressive body-rate and thrust commands via the K matrix.
    constexpr double kMaxPosErr = 2.0;   // metres
    constexpr double kMaxVelErr = 2.0;   // m/s
    for (int i = 0; i < 3; ++i) {
        error(i) = std::max(-kMaxPosErr, std::min(kMaxPosErr, error(i)));
    }
    // Saturate velocity error (indices 7-9)
    for (int i = 7; i < 10; ++i) {
        error(i) = std::max(-kMaxVelErr, std::min(kMaxVelErr, error(i)));
    }
    output = traj_control - gain * error;
    bodyRatesThrustCmd << output(0), output(1), output(2), output(3);

    const bool saturated =
        !raw_error.head<3>().isApprox(error.head<3>()) ||
        !raw_error.segment<3>(7).isApprox(error.segment<3>(7));
    if (saturated) {
        ROS_WARN_THROTTLE(
            0.5,
            "LQR error saturation active: raw_pos_err [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f], "
            "raw_vel_err [%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f]",
            raw_error(0), raw_error(1), raw_error(2),
            error(0), error(1), error(2),
            raw_error(7), raw_error(8), raw_error(9),
            error(7), error(8), error(9));
    }

    if (raw_error.head<3>().norm() > 3.0 || raw_error.segment<3>(7).norm() > 2.5) {
        ROS_WARN_THROTTLE(
            0.5,
            "LQR large tracking error: pos_err_norm=%.2f vel_err_norm=%.2f ref_pos [%.2f, %.2f, %.2f] "
            "cur_pos [%.2f, %.2f, %.2f] cmd [%.2f, %.2f, %.2f, %.2f]",
            raw_error.head<3>().norm(),
            raw_error.segment<3>(7).norm(),
            lqr_quaternion_.getRefStates()(0), lqr_quaternion_.getRefStates()(1), lqr_quaternion_.getRefStates()(2),
            currentPos_[0], currentPos_[1], currentPos_[2],
            bodyRatesThrustCmd(0), bodyRatesThrustCmd(1), bodyRatesThrustCmd(2), bodyRatesThrustCmd(3));
    }
}

void LQR_Controller::publishAttitude(Eigen::Vector4d bodyRatesThrustCmd)
{
    mavros_msgs::AttitudeTarget bodyrateMsg;

    const Eigen::Vector3d requested_rates = bodyRatesThrustCmd.head<3>();
    bodyRatesThrustCmd(0) = std::max(-maxBodyRateXY_, std::min(maxBodyRateXY_, bodyRatesThrustCmd(0)));
    bodyRatesThrustCmd(1) = std::max(-maxBodyRateXY_, std::min(maxBodyRateXY_, bodyRatesThrustCmd(1)));
    bodyRatesThrustCmd(2) = std::max(-maxBodyRateZ_, std::min(maxBodyRateZ_, bodyRatesThrustCmd(2)));

    // When the vehicle is already near the envelope, do not let a tracking
    // transient increase its roll/pitch.  Once outside it, command a bounded
    // body-rate recovery instead of continuing to follow the LQR output.
    if (haveOdom_) {
        const double tilt = std::acos(std::max(-1.0, std::min(1.0,
            std::cos(currentRpy_(0)) * std::cos(currentRpy_(1)))));
        const double soft_tilt = 0.8 * maxTiltRad_;
        if (tilt >= maxTiltRad_) {
            bodyRatesThrustCmd(0) = std::max(-maxBodyRateXY_, std::min(maxBodyRateXY_,
                -tiltRecoveryGain_ * currentRpy_(0)));
            bodyRatesThrustCmd(1) = std::max(-maxBodyRateXY_, std::min(maxBodyRateXY_,
                -tiltRecoveryGain_ * currentRpy_(1)));
            ROS_WARN_THROTTLE(0.5,
                "LQR tilt recovery active: tilt=%.1f deg limit=%.1f deg rpy=[%.1f, %.1f, %.1f] deg",
                tilt * 180.0 / M_PI, maxTiltRad_ * 180.0 / M_PI,
                currentRpy_(0) * 180.0 / M_PI, currentRpy_(1) * 180.0 / M_PI,
                currentRpy_(2) * 180.0 / M_PI);
        } else if (tilt >= soft_tilt) {
            if (currentRpy_(0) * bodyRatesThrustCmd(0) > 0.0) {
                bodyRatesThrustCmd(0) = 0.0;
            }
            if (currentRpy_(1) * bodyRatesThrustCmd(1) > 0.0) {
                bodyRatesThrustCmd(1) = 0.0;
            }
        }
    }

    if (!requested_rates.isApprox(bodyRatesThrustCmd.head<3>())) {
        ROS_WARN_THROTTLE(0.5,
            "LQR body-rate protection active: requested [%.3f, %.3f, %.3f] applied [%.3f, %.3f, %.3f]",
            requested_rates(0), requested_rates(1), requested_rates(2),
            bodyRatesThrustCmd(0), bodyRatesThrustCmd(1), bodyRatesThrustCmd(2));
    }

    double thrust_raw = bodyRatesThrustCmd(3);

    // Compute thrust
    double normalized_thrust = (hoverThrust_ * thrust_raw) / gravity_;
    normalized_thrust = std::max(minNormalizedThrust_, std::min(maxNormalizedThrust_, normalized_thrust));

    if (normalized_thrust <= minNormalizedThrust_ + 1.0e-4 ||
        normalized_thrust >= maxNormalizedThrust_ - 1.0e-4) {
        ROS_WARN_THROTTLE(
            0.5,
            "LQR thrust clamp active: raw_thrust=%.3f normalized=%.3f body_rates [%.3f, %.3f, %.3f]",
            thrust_raw, normalized_thrust,
            bodyRatesThrustCmd(0), bodyRatesThrustCmd(1), bodyRatesThrustCmd(2));
    }

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

    const auto& orientation = msg->pose.pose.orientation;
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    if (q.norm() > 1.0e-6 && q.coeffs().allFinite()) {
        q.normalize();
        const Eigen::Matrix3d rotation = q.toRotationMatrix();
        currentRpy_(0) = std::atan2(rotation(2, 1), rotation(2, 2));
        currentRpy_(1) = std::asin(std::max(-1.0, std::min(1.0, -rotation(2, 0))));
        currentRpy_(2) = std::atan2(rotation(1, 0), rotation(0, 0));
        haveOdom_ = true;
    }

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

        lqr_quaternion_.setStates(msg);
        lqr_quaternion_.computeLQR();
}

void LQR_Controller::trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    if(msg->points.empty()) {
        ROS_WARN("Received empty trajectory");
        return;
    }
    lqr_quaternion_.setTrajectory(*msg);
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
    (void)level;
    applyTuningConfig(config);
}

void LQR_Controller::applyTuningConfig(const lqr_controller::LqrControllerConfig& config)
{
    state_matrix_quat_t Q_quat;
    Q_quat.setZero();
    Q_quat.diagonal() << config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
                         config.Q_quat_w, config.Q_quat_x, config.Q_quat_y, config.Q_quat_z,
                         config.Q_vel_x, config.Q_vel_y, config.Q_vel_z;
    lqr_quaternion_.setQ(Q_quat);

    control_matrix_quat_t R_quat;
    R_quat.setZero();
    R_quat.diagonal() << config.R_rate_x, config.R_rate_y, config.R_rate_z, config.R_thrust;
    lqr_quaternion_.setR(R_quat);

    ROS_INFO("LQR Dynamic Reconfigure: Q_pos [%.1f, %.1f, %.1f], Q_vel [%.1f, %.1f, %.1f], R_rates [%.1f, %.1f, %.1f], R_thrust %.1f",
             config.Q_pos_x, config.Q_pos_y, config.Q_pos_z,
             config.Q_vel_x, config.Q_vel_y, config.Q_vel_z,
             config.R_rate_x, config.R_rate_y, config.R_rate_z, config.R_thrust);
}

void LQR_Controller::loadStaticTuningConfig()
{
    lqr_controller::LqrControllerConfig config;
    privateNodeHandle_.param("Q_pos_x", config.Q_pos_x, 100.0);
    privateNodeHandle_.param("Q_pos_y", config.Q_pos_y, 100.0);
    privateNodeHandle_.param("Q_pos_z", config.Q_pos_z, 80.0);
    privateNodeHandle_.param("Q_quat_w", config.Q_quat_w, 0.5);
    privateNodeHandle_.param("Q_quat_x", config.Q_quat_x, 0.5);
    privateNodeHandle_.param("Q_quat_y", config.Q_quat_y, 0.5);
    privateNodeHandle_.param("Q_quat_z", config.Q_quat_z, 5.0);
    privateNodeHandle_.param("Q_vel_x", config.Q_vel_x, 5.0);
    privateNodeHandle_.param("Q_vel_y", config.Q_vel_y, 5.0);
    privateNodeHandle_.param("Q_vel_z", config.Q_vel_z, 3.0);
    privateNodeHandle_.param("R_rate_x", config.R_rate_x, 15.0);
    privateNodeHandle_.param("R_rate_y", config.R_rate_y, 15.0);
    privateNodeHandle_.param("R_rate_z", config.R_rate_z, 10.0);
    privateNodeHandle_.param("R_thrust", config.R_thrust, 0.4);
    applyTuningConfig(config);
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
