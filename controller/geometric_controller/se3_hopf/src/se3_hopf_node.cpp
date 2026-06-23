// ref: se3_example.cpp
#include "se3_hopf/se3_ctrl.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "se3_hopf_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Se3HopfCtrl *se3_hopf_node = new Se3HopfCtrl(nh, private_nh);
    // se3_example.init(nh);

    ros::spin();

    return 0;
}
