#include <ros/ros.h>
#include "lqr_controller/lqr_controller.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lqr_controller_node");
    ros::NodeHandle nh("~");

    // Load control type from parameter (default: QUATERNION = 0)
    // Set via: rosrun lqr_controller lqr_controller_node _lqr_control_type:=0
    // Or in launch file: <param name="lqr_control_type" value="0" />
    int controlType = 0;
    nh.param("lqr_control_type", controlType, 0);
    if (controlType != 0 && controlType != 1) {
        ROS_WARN("Invalid lqr_control_type %d, defaulting to QUATERNION (0)", controlType);
        controlType = 0;
    }
    nh.setParam("lqr_control_type", controlType);

    ROS_INFO("Starting LQR Controller Node with control type: %s",
              controlType == 0 ? "QUATERNION" : "EULER");

    lqr::LQR_Controller lqrCtrl(nh);

    dynamic_reconfigure::Server<lqr_controller::LqrControllerConfig> srv;
    dynamic_reconfigure::Server<lqr_controller::LqrControllerConfig>::CallbackType f;
    f = boost::bind(&lqr::LQR_Controller::dynamicReconfigureCallback, &lqrCtrl, _1, _2);
    srv.setCallback(f);

    ros::spin();
    return 0;
}