#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "common/printf_utils.h"


using namespace std;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_pos;   // 无人机当前位置
geometry_msgs::PoseStamped aim_pos;    // 无人机目标位置
geometry_msgs::TwistStamped curr_vel;  // 无人机当前速度

void printf_info();
bool is_arrive(geometry_msgs::PoseStamped pos1, geometry_msgs::PoseStamped pos2);

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    //ROS_INFO("current mode: %s",current_state.mode.c_str());
    //ROS_INFO("system_status: %d",current_state.system_status);
}

void arrive_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = *msg;
    //cout <<GREEN <<fabs((*msg).pose.position.z - aim_pos.pose.position.z) <<endl; 
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    curr_vel = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_no_rc");
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

    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity_local", 10, velocityCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0); // 20Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //geometry_msgs::PoseStamped aim_pos;
    aim_pos.pose.position.x = 0;
    aim_pos.pose.position.y = 0;
    aim_pos.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(aim_pos);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int count = 0;

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
            else{
                if(is_arrive(aim_pos,curr_pos)){
                    count++;
                    if(count == 20)  // 10s
                    {
                        aim_pos.pose.position.x = 20;
                        aim_pos.pose.position.y = 0;
                        aim_pos.pose.position.z = 2;
                    }
                    if(count == 40)  // 30s
                    {
                        aim_pos.pose.position.x = 20;
                        aim_pos.pose.position.y = 20;
                        aim_pos.pose.position.z = 2;
                    }
                    if(count == 60)  // 45s
                    {
                        aim_pos.pose.position.x = 0;
                        aim_pos.pose.position.y = 20;
                        aim_pos.pose.position.z = 2;
                    }
                    if(count == 80)  // 60s
                    {
                        aim_pos.pose.position.x = 0;
                        aim_pos.pose.position.y = 0;
                        aim_pos.pose.position.z = 2;
                    }
                    if(count >= 100)  // 75s
                    {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";  // 发送降落命令
                        if( current_state.mode != "AUTO.LAND" && set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
                            ROS_INFO("land enabled");
                        }
                        //任务结束,关闭该节点
                        ros::shutdown();
                    }  
                }
            } // else
        }
        

        local_pos_pub.publish(aim_pos);

        ros::spinOnce();
        printf_info();
        rate.sleep();
    }

    return 0;
}

void printf_info(){
    cout<<YELLOW<<"<<<<<<<<<<<<< Uav Info <<<<<<<<<<<<<<<<\n";
    cout<<GREEN<<"current mode: "<< current_state.mode <<endl;
    cout<<GREEN<<"system status: "<< current_state.system_status <<endl;

    cout<<GREEN<<"uav pos [x,y,z]: "<< curr_pos.pose.position.x <<"[m] " 
        << curr_pos.pose.position.y <<"[m] "<< curr_pos.pose.position.z <<"[m]"<<endl;

    cout<<GREEN<<"Linear Vel [x, y, z]: "<< curr_vel.twist.linear.x <<"[m/s] "
        << curr_vel.twist.linear.y <<"[m/s] "<< curr_vel.twist.linear.z <<"[m/s]"<<endl;

    cout<<GREEN<<"Angular Vel [x, y, z]: "<< curr_vel.twist.angular.x <<"[rad/s] "
        << curr_vel.twist.angular.y <<"[rad/s] "<< curr_vel.twist.angular.z <<"[rad/s]"<<endl;
}

bool is_arrive(geometry_msgs::PoseStamped pos1, geometry_msgs::PoseStamped pos2){
    if(fabs(pos1.pose.position.x - pos2.pose.position.x) < 0.1 &&
        fabs(pos1.pose.position.y - pos2.pose.position.y) < 0.1 && 
        fabs(pos1.pose.position.z - pos2.pose.position.z) < 0.1){
            return true;
        }
    return false;
}
