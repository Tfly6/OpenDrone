#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

using namespace std;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_pos;   // 无人机当前位置
vector<geometry_msgs::PoseStamped> waypoints;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void arrive_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = *msg;
}

void waypoint_cb(const nav_msgs::Path::ConstPtr& msg){
    waypoints.clear();  // 清空之前的航点
    for(int i=0; i<msg->poses.size(); i++){
        waypoints.push_back(msg->poses[i]);
    }
    ROS_INFO("Received %zu waypoints.", waypoints.size());
}

bool is_arrive(geometry_msgs::PoseStamped pos1, geometry_msgs::PoseStamped pos2){
    double distance_threshold = 0.13;  // 到达阈值
    double dx = pos1.pose.position.x - pos2.pose.position.x;
    double dy = pos1.pose.position.y - pos2.pose.position.y;
    double dz = pos1.pose.position.z - pos2.pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance < distance_threshold;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose",10,arrive_pos);
    
    ros::Subscriber waypoint_sub = nh.subscribe<nav_msgs::Path>
        ("/waypoint_generator/waypoints",10,waypoint_cb);
    ros::Publisher traj_trigger = nh.advertise<geometry_msgs::PoseStamped>
        ("/waypoint_generator/traj_start_trigger", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int count = 0;  // 计时
    traj_trigger.publish(curr_pos);

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(waypoints.size() > 0){
            if(is_arrive(curr_pos,pose)){
                pose = waypoints.front();
                waypoints.erase(waypoints.begin());
            }
            if(waypoints.size() == 0 && current_state.armed){
                traj_trigger.publish(curr_pos);
            }   
        }
        
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
