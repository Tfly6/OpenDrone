#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "opendrone/printf_utils.h"


using namespace std;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    ROS_INFO("current mode: %s",current_state.mode.c_str());
    ROS_INFO("system_status: %d",current_state.system_status);
}

geometry_msgs::PoseStamped aim_pos;
geometry_msgs::PoseStamped curr_pos;

void arrive_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = *msg;
    // cout <<GREEN <<"distance: "<<fabs((*msg).pose.position.z - aim_pos.pose.position.z) <<endl; 
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "off_node");
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

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);  // 10Hz 100ms

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("vehicle connected!");

    //geometry_msgs::PoseStamped pose;
    aim_pos.pose.position.x = 0;
    aim_pos.pose.position.y = 0;
    aim_pos.pose.position.z = 2;

    
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(aim_pos);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int count = 0;  // 计时
    bool flag = false;

    while(ros::ok()){
        if( current_state.mode == "OFFBOARD"){
        // if( current_state.mode == "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
            // if( set_mode_client.call(offb_set_mode) &&
            //     offb_set_mode.response.mode_sent){
            //     ROS_INFO("Offboard enabled");
            // }
            if( !current_state.armed){
                if( arming_client.call(arm_cmd) &&arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            }
            // if(!flag){
            //     local_pos_pub.publish(aim_pos);
            //     flag = true;
            // }
            local_pos_pub.publish(aim_pos);
            
            //last_request = ros::Time::now();
        }
        if(current_state.mode == "OFFBOARD" && fabs(curr_pos.pose.position.z - aim_pos.pose.position.z) <= 0.3){
            count++;
            cout <<GREEN << "count: "<< count<< endl;
            if(count > 150) { // 15s
                mavros_msgs::SetMode land_set_mode;
                land_set_mode.request.custom_mode = "AUTO.LAND";  // 发送降落命令
                if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
                    ROS_INFO("land enabled");
                }
                //任务结束,关闭该节点
                ros::shutdown();
            }  
        }
        if(current_state.mode != "OFFBOARD"){
            ROS_INFO("switch to Offboard");
        }

        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
