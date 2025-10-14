/**
 * ref: se3_example.cpp
 * @author tfly
 */

#include "se3_controller/se3_ctrl.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "se3_controller_node");
    ros::NodeHandle nh("~");

    se3Ctrl *se3_controller_node = new se3Ctrl(nh);

    ros::spin();

    return 0;
}