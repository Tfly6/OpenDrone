#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "pid_controller/simple_pid.h"
#include "math_utils/math_utils.h"

#define NUM_MOTOR 4

#define MOTOR_P1 -0.00069
#define MOTOR_P2 0.01271
#define MOTOR_P3 -0.07948
#define MOTOR_P4 0.3052
#define MOTOR_P5 0.008775

#define thrust_max_single_motor 6.0

class pidCtrl {
    public:
        pidCtrl(const ros::NodeHandle &nh);

        void trigger_offboard();
        void trigger_arm();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber pos_sub_;
        ros::Subscriber vel_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber state_sub_;
        ros::Subscriber multiDOFJoint_sub_;
        ros::Subscriber simpleWaypoint_sub_;
        ros::Publisher local_pos_pub_;
        ros::Publisher setpoint_raw_local_pub_;
        ros::Publisher setpoint_raw_attitude_pub_;
        ros::ServiceClient arming_client_;
        ros::ServiceClient set_mode_client_;
        ros::Timer cmdloop_timer_;
        

        // ros::Time last_;
        mavros_msgs::State currState_;
        mavros_msgs::CommandBool arm_cmd;
        vector<geometry_msgs::PoseStamped> waypoints_;
        
        simplePID simpleController;

        bool sim_enable_, arm_triggered_, offboard_triggered_;
        double uavMass_;
        double tilt_max_;
        double yaw_ref_;
        double takeoff_height_;
        Eigen::Vector3d init_pose_;
        Eigen::Vector3d currPose_, currVel_, currAcc_;
        Eigen::Vector3d kp_, ki_, kd_;
        double Kp_x_, Kp_y_, Kp_z_, Ki_x_, Ki_y_, Ki_z_, Kd_x_, Kd_y_, Kd_z_;
        double ff_gain_;
        Eigen::Vector3d integral_, pre_error_, pos_error_max_;
        Eigen::Vector3d targetPos_, targetVel_, targetAcc_;
        Eigen::Vector4d targetAtt_;
        double targetThrust_;
        double gravity_ = 9.8;

        enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED, TEST } node_state_;
        enum ControlType {SIMPLE_PID, NONE} pid_type_;

        void computeTarget(const double dt);
        // void compute(Eigen::Vector4d &targetAtt, double dt);
        Eigen::Vector3d accelToThrust(const Eigen::Vector3d& accel_sp, double mass, double tilt_max);
        Eigen::Vector3d thrustToThrottle(const Eigen::Vector3d& thrust_sp);
        Eigen::Vector4d ThrottleToAttitude(const Eigen::Vector3d& thr_sp, double yaw_sp);

        void pubLocalPose(const Eigen::Vector3d &pose); 
        void pubAttitudeTarget(const Eigen::Vector4d &target_attitude, const double thrust_des);

        void controlLoop(const ros::TimerEvent &event);
        void simpleWaypoint_cb(const nav_msgs::Path::ConstPtr& msg);
        void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void pos_cb(const geometry_msgs::PoseStamped &msg);
        void vel_cb(const geometry_msgs::TwistStamped &msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

};

#endif