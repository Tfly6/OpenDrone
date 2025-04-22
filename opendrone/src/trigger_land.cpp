#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <std_srvs/SetBool.h>

ros::ServiceClient land_client;
int trigger_channel = 4; // channel 5
int trigger_threshold = 1900;
bool last_trigger_state; // 上一次触发状态（用于防抖）

void rcCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
    // 检查通道索引是否有效
    if (msg->channels.size() <= trigger_channel) {
        ROS_WARN("Trigger channel %d does not exist!", trigger_channel);
        return;
    }

    // 获取当前通道值
    int current_value = msg->channels[trigger_channel];
    bool current_trigger_state = (current_value > trigger_threshold);

    // 检测上升沿（从低到高变化）
    if (current_trigger_state && !last_trigger_state) {
        ROS_INFO("Trigger detected! Sending land request...");

        // 构造服务请求
        std_srvs::SetBool srv;
        srv.request.data = true;

        // 调用服务
        land_client.call(srv);
        // if (land_client.call(srv)) {
        //     if (srv.response.success) {
        //         ROS_INFO("Land command accepted!");
        //     } else {
        //         ROS_WARN("Land command rejected: %s", srv.response.message.c_str());
        //     }
        // } else {
        //     ROS_ERROR("Failed to call land service!");
        // }
    }

    // 更新上一次状态
    last_trigger_state = current_trigger_state;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "land_client");
    ros::NodeHandle nh;

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, &rcCallback);

    land_client = nh.serviceClient<std_srvs::SetBool>("land");

    ros::spin();
    return 0;
}