/**
 * @author tfly
 */

#include "pid_controller/pid_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pidCtrl");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pidCtrl *pidCtrl_node = new pidCtrl(nh, private_nh);

    ros::spin();

    return 0;
}
