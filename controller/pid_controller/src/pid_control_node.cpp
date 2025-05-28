#include "pid_controller/pid_control.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pidCtrl");
    ros::NodeHandle nh("~");
    pidCtrl *pidCtrl_node = new pidCtrl(nh);

    dynamic_reconfigure::Server<pid_controller::PidControllerConfig> srv;
    dynamic_reconfigure::Server<pid_controller::PidControllerConfig>::CallbackType f;
    f = boost::bind(&pidCtrl::dynamicReconfigureCallback, pidCtrl_node, _1, _2);
    srv.setCallback(f);

    ros::spin();

    return 0;
}