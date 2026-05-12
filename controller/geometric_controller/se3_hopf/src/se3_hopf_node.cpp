<<<<<<< HEAD:controller/se3_controller/src/se3_controller_node.cpp
/**
 * ref: se3_example.cpp
 * @author tfly
 */

#include "se3_controller/se3_ctrl.h"
=======
// ref: se3_example.cpp
#include "se3_hopf/se3_ctrl.h"
>>>>>>> origin/dev:controller/geometric_controller/se3_hopf/src/se3_hopf_node.cpp

int main(int argc, char **argv){

    ros::init(argc, argv, "se3_hopf_node");
    ros::NodeHandle nh("~");

<<<<<<< HEAD:controller/se3_controller/src/se3_controller_node.cpp
    se3Ctrl *se3_controller_node = new se3Ctrl(nh);
=======
    Se3HopfCtrl *se3_hopf_node = new Se3HopfCtrl(nh);
    // se3_example.init(nh);
>>>>>>> origin/dev:controller/geometric_controller/se3_hopf/src/se3_hopf_node.cpp

    ros::spin();

    return 0;
}