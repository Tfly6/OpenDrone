#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "mav_trajectory_generation_ros/trajectory_generation.h"

// std_msgs::Int8 state;
// void state_cb(const std_msgs::Int8::ConstPtr &msg)
// {
//     state = *msg;
// }

int main(int argc, char** argv) {

  ros::init(argc, argv, "planner");

  ros::NodeHandle n;
  TrajectoryGeneration planner(n);
  ROS_INFO("Initialized trajectory.");
  // ros::Subscriber state_sub = n.subscribe<std_msgs::Int8>("geometric_controller/state", 10, state_cb);

  // while(ros::ok()){
  //   if(state.data == 1){
  //       planner.triggerWaypoints();
  //       // ros::Duration(10).sleep();
  //       break;
  //   }
  //   ros::Duration(0.5).sleep();
  //   ros::spinOnce();
  // }
  // ROS_WARN_STREAM("DONE. GOODBYE.");
  ros::spin();
  return 0;
}