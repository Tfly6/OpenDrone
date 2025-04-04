#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "opendrone/trajectory_generation.h"

std_msgs::Int8 state;
void state_cb(const std_msgs::Int8::ConstPtr &msg)
{
    state = *msg;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "planner");

  ros::NodeHandle n;
  TrajectoryGeneration planner(n);
  ros::Subscriber state_sub = n.subscribe<std_msgs::Int8>("geometric_controller/state", 10, state_cb);
//   ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
//   ros::Duration(5.0).sleep();
//   ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  Eigen::Vector3d position, velocity;
  position << 4.0, 5.0, 6.0;
  velocity << 0.0, 0.0, 0.0;

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
//   std::cin.get();
//   for (int i = 0; i < 10; i++) {
//     ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
//   }
  while(ros::ok()){
    if(state.data == 1){
        break;
    }

    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  mav_trajectory_generation::Trajectory trajectory;
    planner.planTrajectory(position, velocity, &trajectory);
    planner.publishTrajectory(trajectory);
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}