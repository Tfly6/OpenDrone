#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "pid_controller/simple_pid.h"
#include "pid_controller/cascade_pid.h"
#include "pid_controller/PidControllerConfig.h"
#include "math_utils/math_utils.h"

#define SIMPLE_PID 1
#define CASCADE_PID 2

class pidCtrl {
    public:
        pidCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);

        void dynamicReconfigureCallback(pid_controller::PidControllerConfig &config, uint32_t level);
        void TrySetOffboard(const ros::Time &now);
        void TryArm(const ros::Time &now);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        dynamic_reconfigure::Server<pid_controller::PidControllerConfig> dyn_config_server_;
        dynamic_reconfigure::Server<pid_controller::PidControllerConfig>::CallbackType dyn_config_cb_type_;
        ros::Subscriber pos_sub_, vel_sub_, state_sub_;
        ros::Subscriber multiDOFJoint_sub_, simpleWaypoint_sub_;

        ros::Publisher local_pos_pub_, vel_pub_;
        ros::Publisher setpoint_raw_local_pub_;
        ros::Publisher setpoint_raw_attitude_pub_;
        ros::Publisher referencePosePub_;
        ros::Publisher referenceVelPub_;
        ros::Publisher referenceAccPub_;
        ros::Publisher flight_state_pub_;

        ros::ServiceClient arming_client_;
        ros::ServiceClient set_mode_client_;
        ros::ServiceServer land_service_;
        ros::Timer cmdloop_timer_;
        

        // ros::Time last_;
        mavros_msgs::State currState_;
        mavros_msgs::CommandBool arm_cmd;
        vector<geometry_msgs::PoseStamped> waypoints_;
        
        simplePID simpleController;
        cascadePID cascadeController;

        bool sim_enable_;
        bool landing_locked_{false};
        bool enable_auto_offboard_{false};
        bool enable_auto_arm_{false};
        bool autoTakeoff_{false};
        int pid_type_;
        int offboard_warmup_counter_{0};
        int offboard_warmup_count_{80};
        double request_interval_{1.0};
        ros::Time last_mode_request_;
        ros::Time last_arm_request_;
        double uavMass_;
        // double tilt_max_;
        double yaw_ref_;
        double takeoff_height_;
        Eigen::Vector3d init_pose_;
        Eigen::Vector3d currPose_, currVel_;
        // Eigen::Vector3d kp_, ki_, kd_;
        // double Kp_x_, Kp_y_, Kp_z_, Ki_x_, Ki_y_, Ki_z_, Kd_x_, Kd_y_, Kd_z_;
        
        Eigen::Vector3d geo_fence_;
        Eigen::Vector3d targetPos_, targetVel_, targetAcc_;
        Eigen::Vector4d targetAtt_;
        double targetThrust_;
        // double gravity_ = 9.8;

        enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED, EMERGENCY } flightState_, prev_flightState_;
        // enum ControlType {SIMPLE_PID, CASCADE_PID, NONE} pid_type_;

        std::string state2string(FlightState state) {
            switch (state) {
            case WAITING_FOR_CONNECTED:
                return "WAITING_FOR_CONNECTED";
            case WAITING_FOR_OFFBOARD:
                return "WAITING_FOR_OFFBOARD";
            case MISSION_EXECUTION:
                return "MISSION_EXECUTION";
            case LANDING:
                return "LANDING";
            case LANDED:
                return "LANDED";
            case TAKEOFF:
                return "TAKEOFF";
            case EMERGENCY:
                return "EMERGENCY";
            default:
                return "UNKNOWN_STATE";
            }
        }

        void computeTarget(const double dt);
       
        void pubVel(const Eigen::Vector3d &vel); 
        void pubLocalPose(const Eigen::Vector3d &pose); 
        void pubAttitudeTarget(const Eigen::Vector4d &target_attitude, const double thrust_des);

        void controlLoop(const ros::TimerEvent &event);
        bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
        void simpleWaypoint_cb(const nav_msgs::Path::ConstPtr& msg);
        void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg);
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        // void acc_cb(const geometry_msgs::AccelWithCovarianceStamped &msg);

};

#endif
