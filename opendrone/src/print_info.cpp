/**
 * @author tfly
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include "opendrone/printf_utils.h"

using namespace std;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_pos;
geometry_msgs::TwistStamped curr_vel;
nav_msgs::Odometry vins_pose;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void arrive_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = *msg;
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    curr_vel = *msg;
}

void vins_cb(const nav_msgs::Odometry::ConstPtr& msg){
    vins_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "print_info");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",10,arrive_pos);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_local", 10, vel_cb);
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>
            ("/vins_estimator/odometry", 100, vins_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        cout<<YELLOW<<"<<<<<<<<<<<<< Uav Info <<<<<<<<<<<<<<<<\n";
        cout<<GREEN<<"connected: "<< unsigned(current_state.connected) <<endl;
        cout<<GREEN<<"current mode: "<< current_state.mode <<endl;
        cout<<GREEN<<"Arming: "<< unsigned(current_state.armed) <<endl;
        cout<<GREEN<<"system status: "<< unsigned(current_state.system_status) <<endl;

        cout<<GREEN<<"vins pos [x.y,z]: "<< vins_pose.pose.pose.position.x <<"[m] " 
            << vins_pose.pose.pose.position.y <<"[m] "<< vins_pose.pose.pose.position.z <<"[m]"<<endl;
            
        cout<<GREEN<<"uav pos [x,y,z]: "<< curr_pos.pose.position.x <<"[m] " 
            << curr_pos.pose.position.y <<"[m] "<< curr_pos.pose.position.z <<"[m]"<<endl;

        cout<<GREEN<<"Linear Vel [x, y, z]: "<< curr_vel.twist.linear.x <<"[m/s] "
            << curr_vel.twist.linear.y <<"[m/s] "<< curr_vel.twist.linear.z <<"[m/s]"<<endl;

        cout<<GREEN<<"Angular Vel [x, y, z]: "<< curr_vel.twist.angular.x <<"[rad/s] "
            << curr_vel.twist.angular.y <<"[rad/s] "<< curr_vel.twist.angular.z <<"[rad/s]"<<endl;
            
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
