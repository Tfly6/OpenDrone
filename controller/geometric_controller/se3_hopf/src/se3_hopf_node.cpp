// ref: se3_example.cpp
#include "se3_hopf/se3_ctrl.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "se3_hopf_node");
    ros::NodeHandle nh("~");

    Se3HopfCtrl *se3_hopf_node = new Se3HopfCtrl(nh);
    // se3_example.init(nh);

    ros::spin();

    return 0;
}