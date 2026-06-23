#include <ros/ros.h>
#include "lqr_controller/lqr_controller.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lqr_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Starting LQR Controller Node with quaternion control");

    lqr::LQR_Controller lqrCtrl(nh, private_nh);

    ros::spin();
    return 0;
}
