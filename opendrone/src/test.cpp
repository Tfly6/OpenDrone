#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include "controller_msgs/FlatTarget.h"

std_msgs::Int8 state;
geometry_msgs::TwistStamped local_vel;
geometry_msgs::PoseStamped local_pos;

void state_cb(const std_msgs::Int8::ConstPtr &msg)
{
    state = *msg;
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    local_vel = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    float mav_acc = nh.param<float>("max_acc", 9.0);
    ros::Rate rate(2);

    ros::Subscriber state_sub = nh.subscribe<std_msgs::Int8>("geometric_controller/state", 10, state_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 10, local_vel_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 10);
    controller_msgs::FlatTarget target;
    while(ros::ok()){
        ROS_INFO_STREAM("data: "<< unsigned(state.data));
        if(state.data == 1 && fabs(local_pos.pose.position.z - 2.0) < 0.2){
            // target.header.stamp = ros::Time::now();
            target.type_mask = 4;
            target.position.x = 10;
            target.position.y = 0;
            target.position.z = 2;
            local_pos_pub.publish(target);
        }
        ros::spinOnce();
        rate.sleep();
    }
}