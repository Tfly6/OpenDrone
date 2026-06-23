#pragma once

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "lqr_controller/lqr_quaternion.hpp"
#include "lqr_controller/LqrControllerConfig.h"

namespace lqr {

class LQR_Controller {
  public:
    LQR_Controller(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);
    virtual ~LQR_Controller();

    void controlLoop(const ros::TimerEvent& event);
    bool landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    void dynamicReconfigureCallback(lqr_controller::LqrControllerConfig &config, uint32_t level);

  private:
    enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED, EMERGENCY };

    std::string state2string(FlightState state);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
    void TrySetOffboard(const ros::Time& now);
    void TryArm(const ros::Time& now);
    void computeControlCommands(Eigen::Vector4d& bodyRatesThrustCmd);
    void publishAttitude(Eigen::Vector4d bodyRatesThrustCmd);
    bool isAtPosition(const Eigen::Vector3d& target, double threshold);
    void applyTuningConfig(const lqr_controller::LqrControllerConfig& config);
    void loadStaticTuningConfig();

    // ROS interfaces
    ros::NodeHandle nodeHandle_;
    ros::NodeHandle privateNodeHandle_;
    dynamic_reconfigure::Server<lqr_controller::LqrControllerConfig> dynConfigServer_;
    dynamic_reconfigure::Server<lqr_controller::LqrControllerConfig>::CallbackType dynConfigCallbackType_;
    ros::Subscriber stateSub_;
    ros::Subscriber odomSub_;
    ros::Subscriber trajectorySub_;
    ros::Publisher attitudePub_;
    ros::Publisher localPosPub_;
    ros::Publisher referencePosePub_;
    ros::Publisher referenceVelPub_;
    ros::Publisher referenceAccPub_;
    ros::Publisher flightStatePub_;
    ros::ServiceClient armingClient_;
    ros::ServiceClient setModeClient_;
    ros::ServiceServer landService_;
    ros::Timer controlTimer_;

    // LQR controller
    lqr::LQR_Quaternion lqr_quaternion_;

    // State machine
    FlightState flightState_;
    FlightState prevFlightState_;

    // Flags
    bool simEnable_{false};
    bool takeoffComplete_{false};
    bool landingLocked_{false};
    bool enableAutoOffboard_{false};
    bool enableAutoArm_{false};
    bool autoTakeoff_{false};
    bool useDynamicReconfigure_{false};
    int offboardWarmupCounter_;
    int offboardWarmupCount_;
    double requestInterval_;
    ros::Time lastModeRequest_;
    ros::Time lastArmRequest_;

    // Parameters
    double takeoffHeight_;
    double mass_;
    double gravity_{9.81};
    double hoverThrust_;
    Eigen::Vector3d initPose_;
    Eigen::Vector3d geoFence_;
    Eigen::Vector3d currentPos_;
    Eigen::Vector3d targetPos_;

    // Current mavros state
    mavros_msgs::State currentState_;
};

}  // namespace lqr
