#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <cmath>

nav_msgs::Path point()
{
    // Circle parameters
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    double h = 1.0;
    double scale = 7.0;

    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.25;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y =  scale * 0.5;
    pt.pose.position.x =  scale * 5.3;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.75;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 0.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      
    // Return
    return waypoints;
}

// Circle trajectory
nav_msgs::Path circle()
{
    double radius = 5.0;  // 圆半径
    double h = 2.0;       // 飞行高度
    int num_points = 30; // 采样点数（越大越平滑）
    double center_x = 0.0; // 圆心X坐标
    double center_y = 0.0; // 圆心Y坐标

    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    // x=center_x+rcos(θ)
    // y=center_y+rsin(θ)
    // 生成圆轨迹
    for (int i = 0; i < num_points; ++i) 
    {
        double theta = 2 * M_PI * i / num_points; // 角度参数
        double x = center_x + radius * cos(theta);
        double y = center_y + radius * sin(theta);

        pt.pose.position.x = x;
        pt.pose.position.y = y;
        pt.pose.position.z = h;

        // 计算航向角（沿切线方向）
        double dx_dt = -radius * sin(theta); // 导数
        double dy_dt =  radius * cos(theta);
        double yaw = atan2(dy_dt, dx_dt); // 切线方向的航向角
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        waypoints.poses.push_back(pt);
    }
    
    if (num_points > 0) 
    {
        // 将第一个点添加到末尾，确保闭合
        pt = waypoints.poses[0];
        waypoints.poses.push_back(pt);
        pt = waypoints.poses[1];
        waypoints.poses.push_back(pt);
        pt = waypoints.poses[2];
        // waypoints.poses.push_back(pt);
        // pt = waypoints.poses[3];
        // waypoints.poses.push_back(pt);
        // 飞回圆心
        pt.pose.position.y =  0.0 + center_x;
        pt.pose.position.x =  0.0 + center_y;
        pt.pose.position.z =  h;
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        waypoints.poses.push_back(pt);
    }

    return waypoints;
}

nav_msgs::Path eight()
{
    double offset_x = 0.0;  // 轨迹整体偏移
    double offset_y = 0.0;
    double r = 10.0;        // 半径
    double h = 2.0;         // 高度
    int num_points = 30;   // 循环的采样点数

    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;

    // 生成一个8字循环（逆时针）
    // 8字轨迹参数方程
    // x(t)=rcos(t)
    // y(t)=rsin(2t)
    for (int i = 0; i < num_points; ++i) {
        double t = i * 2 * M_PI / num_points;  // 时间参数
        double x = r * cos(t) + offset_x;
        double y = r * sin(2 * t) + offset_y;
        double z = h;

        pt.pose.position.x = x;
        pt.pose.position.y = y;
        pt.pose.position.z = z;
        // 计算航向角（示例：沿轨迹切线方向）
        double dx_dt = -r * sin(t);
        double dy_dt = 2 * r * cos(2 * t);
        double yaw = atan2(dy_dt, dx_dt);
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        waypoints.poses.push_back(pt);
    }
    return waypoints;
}

// nav_msgs::Path circle()
// {
//     double h = 1.0;
//     double scale = 5.0;
//     nav_msgs::Path waypoints;
//     geometry_msgs::PoseStamped pt;
//     pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
//     pt.pose.position.y = -1.2 * scale;
//     pt.pose.position.x =  2.5 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);      

//     pt.pose.position.y = -2.4 * scale;
//     pt.pose.position.x =  5.0 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);      
//     pt.pose.position.y =  0.0 * scale;
//     pt.pose.position.x =  5.0 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);  
    
//     pt.pose.position.y = -1.2 * scale;
//     pt.pose.position.x =  2.5 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);      

//     pt.pose.position.y = -2.4 * scale;
//     pt.pose.position.x =  0. * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);  
//     pt.pose.position.y =  0.0 * scale;
//     pt.pose.position.x =  0.0 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);       

//     pt.pose.position.y = -1.2 * scale;
//     pt.pose.position.x =  2.5 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);      

//     pt.pose.position.y = -2.4 * scale;
//     pt.pose.position.x =  5.0 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);      
//     pt.pose.position.y =  0.0 * scale;
//     pt.pose.position.x =  5.0 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);  
    
//     pt.pose.position.y = -1.2 * scale;
//     pt.pose.position.x =  2.5 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);      

//     pt.pose.position.y = -2.4 * scale;
//     pt.pose.position.x =  0. * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);  
//     pt.pose.position.y =  0.0 * scale;
//     pt.pose.position.x =  0.0 * scale;
//     pt.pose.position.z =  h;
//     waypoints.poses.push_back(pt);     

//     // Return
//     return waypoints;
// }


// Figure 8 trajectory
// nav_msgs::Path eight()
// {
//     // Circle parameters
//     double offset_x = 0.0;
//     double offset_y = 0.0;
//     double r = 10.0;
//     double h = 2.0;
//     nav_msgs::Path waypoints;
//     geometry_msgs::PoseStamped pt;
//     pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    

//     for(int i=0; i< 1; ++i)
//     {
//         // First loop
//         pt.pose.position.x =  r + offset_x;
//         pt.pose.position.y = -r + offset_y;
//         pt.pose.position.z =  h/2;
//         waypoints.poses.push_back(pt);      
//         pt.pose.position.x =  r*2 + offset_x * 2;
//         pt.pose.position.y =  0 ;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  r*3 + offset_x * 3;
//         pt.pose.position.y =  r ;
//         pt.pose.position.z =  h/2;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  r*4 + offset_x * 4;
//         pt.pose.position.y =  0 ;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);       
//         pt.pose.position.x =  r*3 + offset_x * 3;
//         pt.pose.position.y = -r ;
//         pt.pose.position.z =  h/2;
//         waypoints.poses.push_back(pt);      
//         pt.pose.position.x =  r*2 + offset_x * 2;
//         pt.pose.position.y =  0 ;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  r + offset_x * 2;
//         pt.pose.position.y =  r ;
//         pt.pose.position.z =  h/2;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  0  + offset_x;
//         pt.pose.position.y =  0;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);
//         // Second loop
//         pt.pose.position.x =  r + offset_x;
//         pt.pose.position.y = -r;
//         pt.pose.position.z =  h / 2 * 3;
//         waypoints.poses.push_back(pt);      
//         pt.pose.position.x =  r*2 + offset_x * 2;
//         pt.pose.position.y =  0;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  r*3 + offset_x * 3;
//         pt.pose.position.y =  r;
//         pt.pose.position.z =  h / 2 * 3;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  r*4 + offset_x * 4;
//         pt.pose.position.y =  0;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);       
//         pt.pose.position.x =  r*3 + offset_x * 3;
//         pt.pose.position.y = -r;
//         pt.pose.position.z =  h / 2 * 3;
//         waypoints.poses.push_back(pt);      
//         pt.pose.position.x =  r*2 + offset_x * 2;
//         pt.pose.position.y =  0;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  r + offset_x;
//         pt.pose.position.y =  r + offset_y;
//         pt.pose.position.z =  h / 2 * 3;
//         waypoints.poses.push_back(pt);  
//         pt.pose.position.x =  0;
//         pt.pose.position.y =  0;
//         pt.pose.position.z =  h;
//         waypoints.poses.push_back(pt);  
//     }
//     return waypoints;   
// }  
#endif