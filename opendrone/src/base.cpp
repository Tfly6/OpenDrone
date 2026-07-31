#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <opendrone/PlannerOutput.h>
#include <opendrone/PlannerOutputPoint.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>

class BasePositionController {
public:
    BasePositionController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh)
    {
        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, &BasePositionController::stateCallback, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose", 10, &BasePositionController::poseCallback, this);
        planner_output_sub_ = nh_.subscribe<opendrone::PlannerOutput>(
            "/planner/output", 10, &BasePositionController::plannerOutputCallback, this);

        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
        flight_state_pub_ = nh_.advertise<std_msgs::Int8>("/flight_state", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        land_service_ = nh_.advertiseService("/land", &BasePositionController::landCallback, this);

        private_nh_.param<bool>("enable_sim", sim_enable_, false);
        private_nh_.param<bool>("enable_auto_offboard", enable_auto_offboard_, sim_enable_);
        private_nh_.param<bool>("enable_auto_arm", enable_auto_arm_, sim_enable_);
        private_nh_.param<bool>("auto_takeoff", auto_takeoff_, true);
        private_nh_.param<int>("offboard_warmup_count", offboard_warmup_count_, 80);
        private_nh_.param<double>("request_interval", request_interval_, 1.0);
        private_nh_.param<double>("takeoff_height", takeoff_height_, 2.0);
        private_nh_.param<double>("arrival_threshold", arrival_threshold_, 0.10);
        private_nh_.param<double>("publish_rate", publish_rate_, 50.0);
        private_nh_.param<double>("geo_fence/x", geo_fence_x_, 10.0);
        private_nh_.param<double>("geo_fence/y", geo_fence_y_, 10.0);
        private_nh_.param<double>("geo_fence/z", geo_fence_z_, 7.0);

        if (offboard_warmup_count_ < 1) {
            offboard_warmup_count_ = 1;
        }
        if (request_interval_ < 0.1) {
            request_interval_ = 0.1;
        }
        if (publish_rate_ < 2.0) {
            publish_rate_ = 2.0;
        }

        target_pose_.header.frame_id = "map";
        target_pose_.pose.position.x = 0.0;
        target_pose_.pose.position.y = 0.0;
        target_pose_.pose.position.z = takeoff_height_;
        setYaw(target_pose_, 0.0);

        flight_state_ = WAITING_FOR_CONNECTED;
        prev_flight_state_ = flight_state_;
        last_mode_request_ = ros::Time(0);
        last_arm_request_ = ros::Time(0);

        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_), &BasePositionController::controlLoop, this);
    }

private:
    enum FlightState {
        WAITING_FOR_CONNECTED = 0,
        WAITING_FOR_OFFBOARD = 1,
        TAKEOFF = 2,
        MISSION_EXECUTION = 3,
        LANDING = 4,
        LANDED = 5,
        EMERGENCY = 6,
    };

    static void setYaw(geometry_msgs::PoseStamped& pose, const double yaw)
    {
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw * 0.5);
    }

    static std::string stateToString(const FlightState state)
    {
        switch (state) {
        case WAITING_FOR_CONNECTED:
            return "WAITING_FOR_CONNECTED";
        case WAITING_FOR_OFFBOARD:
            return "WAITING_FOR_OFFBOARD";
        case TAKEOFF:
            return "TAKEOFF";
        case MISSION_EXECUTION:
            return "MISSION_EXECUTION";
        case LANDING:
            return "LANDING";
        case LANDED:
            return "LANDED";
        case EMERGENCY:
            return "EMERGENCY";
        default:
            return "UNKNOWN_STATE";
        }
    }

    void controlLoop(const ros::TimerEvent&)
    {
        publishFlightState();

        if (flight_state_ != prev_flight_state_) {
            ROS_WARN_STREAM("base position baseline state changed from "
                            << stateToString(prev_flight_state_) << " to "
                            << stateToString(flight_state_));
            prev_flight_state_ = flight_state_;
        }

        switch (flight_state_) {
        case WAITING_FOR_CONNECTED:
            ROS_INFO_ONCE("base position baseline waiting for FCU connection...");
            if (current_state_.connected) {
                offboard_warmup_counter_ = 0;
                flight_state_ = WAITING_FOR_OFFBOARD;
            }
            break;

        case WAITING_FOR_OFFBOARD:
            ROS_INFO_ONCE("base position baseline waiting for OFFBOARD mode and arming...");
            publishTargetPose();
            ++offboard_warmup_counter_;
            trySetOffboard(ros::Time::now());
            tryArm(ros::Time::now());
            if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
                if (auto_takeoff_) {
                    setTakeoffTarget();
                    flight_state_ = TAKEOFF;
                } else {
                    flight_state_ = MISSION_EXECUTION;
                }
            }
            break;

        case TAKEOFF:
            ROS_INFO_ONCE("base position baseline taking off with PX4 position controller...");
            publishTargetPose();
            if (has_pose_ && distanceToTarget() < arrival_threshold_) {
                flight_state_ = MISSION_EXECUTION;
            }
            break;

        case MISSION_EXECUTION:
            ROS_INFO_ONCE("base position baseline executing mission...");
            publishTargetPose();
            break;

        case LANDING:
            requestAutoLand();
            break;

        case LANDED:
            if (!current_state_.armed) {
                ROS_INFO_ONCE("base position baseline landed and disarmed.");
            }
            break;

        case EMERGENCY:
            ROS_ERROR_THROTTLE(1.0, "base position baseline emergency, requesting land.");
            flight_state_ = LANDING;
            break;
        }
    }

    void publishTargetPose()
    {
        target_pose_.header.stamp = ros::Time::now();
        local_pos_pub_.publish(target_pose_);
    }

    void publishFlightState()
    {
        std_msgs::Int8 msg;
        msg.data = static_cast<int8_t>(flight_state_);
        flight_state_pub_.publish(msg);
    }

    void setTakeoffTarget()
    {
        if (has_pose_) {
            target_pose_.pose.position.x = current_pose_.pose.position.x;
            target_pose_.pose.position.y = current_pose_.pose.position.y;
        }
        target_pose_.pose.position.z = takeoff_height_;
    }

    double distanceToTarget() const
    {
        const double dx = current_pose_.pose.position.x - target_pose_.pose.position.x;
        const double dy = current_pose_.pose.position.y - target_pose_.pose.position.y;
        const double dz = current_pose_.pose.position.z - target_pose_.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void trySetOffboard(const ros::Time& now)
    {
        if (landing_locked_ || !enable_auto_offboard_) {
            return;
        }
        if (current_state_.mode == "OFFBOARD") {
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
            ROS_INFO_THROTTLE(2.0, "base position baseline requested OFFBOARD mode.");
        } else {
            ROS_WARN_THROTTLE(2.0, "base position baseline failed to request OFFBOARD mode.");
        }
        last_mode_request_ = now;
    }

    void tryArm(const ros::Time& now)
    {
        if (landing_locked_ || !enable_auto_arm_) {
            return;
        }
        if (current_state_.armed) {
            return;
        }
        if (enable_auto_offboard_ && current_state_.mode != "OFFBOARD") {
            return;
        }
        if ((now - last_arm_request_).toSec() < request_interval_) {
            return;
        }

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO_THROTTLE(2.0, "base position baseline requested arming.");
        } else {
            ROS_WARN_THROTTLE(2.0, "base position baseline failed to arm.");
        }
        last_arm_request_ = now;
    }

    void requestAutoLand()
    {
        landing_locked_ = true;
        if (current_state_.mode == "AUTO.LAND") {
            flight_state_ = LANDED;
            return;
        }

        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if (set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent) {
            ROS_INFO("base position baseline requested AUTO.LAND.");
            flight_state_ = LANDED;
        } else {
            ROS_WARN_THROTTLE(2.0, "base position baseline failed to request AUTO.LAND.");
        }
    }

    bool landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
    {
        if (request.data) {
            landing_locked_ = true;
            flight_state_ = LANDING;
            response.success = true;
            response.message = "base position baseline landing requested";
        } else {
            response.success = false;
            response.message = "base position baseline ignores land=false";
        }
        return true;
    }

    void plannerOutputCallback(const opendrone::PlannerOutput::ConstPtr& msg)
    {
        if (msg->points.empty()) {
            ROS_WARN_THROTTLE(2.0, "base position baseline received empty planner output.");
            return;
        }

        const opendrone::PlannerOutputPoint* selected = nullptr;
        for (const auto& point : msg->points) {
            if (point.valid_mask & opendrone::PlannerOutputPoint::VALID_POSITION) {
                selected = &point;
                break;
            }
        }
        if (!selected) {
            ROS_WARN_THROTTLE(2.0, "base position baseline received planner output without position.");
            return;
        }

        target_pose_.pose.position.x = selected->position.x;
        target_pose_.pose.position.y = selected->position.y;
        target_pose_.pose.position.z = selected->position.z;
        if (selected->valid_mask & opendrone::PlannerOutputPoint::VALID_YAW) {
            setYaw(target_pose_, selected->yaw);
        }
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
        if (flight_state_ == MISSION_EXECUTION && !current_state_.armed) {
            flight_state_ = EMERGENCY;
            landing_locked_ = true;
            ROS_ERROR("base position baseline: unexpected disarm during mission.");
        }
        if (current_state_.mode == "AUTO.LAND" && !landing_locked_) {
            landing_locked_ = true;
            ROS_WARN("base position baseline landing lock enabled (AUTO.LAND detected).");
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = *msg;
        if (!has_pose_) {
            has_pose_ = true;
            target_pose_.pose.position.x = current_pose_.pose.position.x;
            target_pose_.pose.position.y = current_pose_.pose.position.y;
        }

        const double x = current_pose_.pose.position.x;
        const double y = current_pose_.pose.position.y;
        const double z = current_pose_.pose.position.z;
        const bool out_of_fence =
            std::fabs(x) > geo_fence_x_ || std::fabs(y) > geo_fence_y_ || z > geo_fence_z_ || z < -0.2;
        if (out_of_fence && flight_state_ != LANDING && flight_state_ != LANDED) {
            flight_state_ = EMERGENCY;
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber planner_output_sub_;
    ros::Publisher local_pos_pub_;
    ros::Publisher flight_state_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceServer land_service_;
    ros::Timer control_timer_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped target_pose_;

    FlightState flight_state_;
    FlightState prev_flight_state_;

    bool sim_enable_{false};
    bool enable_auto_offboard_{false};
    bool enable_auto_arm_{false};
    bool auto_takeoff_{true};
    bool has_pose_{false};
    bool landing_locked_{false};
    int offboard_warmup_counter_{0};
    int offboard_warmup_count_{80};
    double request_interval_{1.0};
    double takeoff_height_{2.0};
    double arrival_threshold_{0.10};
    double publish_rate_{50.0};
    double geo_fence_x_{10.0};
    double geo_fence_y_{10.0};
    double geo_fence_z_{7.0};
    ros::Time last_mode_request_;
    ros::Time last_arm_request_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    BasePositionController controller(nh, private_nh);
    ros::spin();
    return 0;
}
