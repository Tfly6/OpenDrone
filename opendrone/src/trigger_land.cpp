/**
 * @author tfly
 */

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <std_srvs/SetBool.h>

ros::ServiceClient land_client;
int trigger_channel = 5; // channel 6
int trigger_threshold = 1999;
int last_value = 1999;
int current_value = 0;
bool is_change = false;
bool is_trigger = false;

void rcCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
    // 检查通道索引是否有效
    if (msg->channels.size() <= trigger_channel) {
        ROS_WARN("Trigger channel %d does not exist!", trigger_channel);
        return;
    }

    
    current_value = msg->channels[trigger_channel];

    // 获取当前通道值
    if(current_value != last_value){
        is_change = true;
    }
    if(is_change && (current_value == trigger_threshold) && !is_trigger){
        ROS_INFO("Trigger detected! Sending land request...");

        // 构造服务请求
        std_srvs::SetBool srv;
        srv.request.data = true;

        // 调用服务
        land_client.call(srv);
        is_trigger = true;
    }
    is_change = false;
    last_value = current_value;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "land_client");
    ros::NodeHandle nh;

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, &rcCallback);

    land_client = nh.serviceClient<std_srvs::SetBool>("/land");

    ros::spin();
    return 0;
}