#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <cmath>

#include <geometry_msgs/Pose.h>

inline geometry_msgs::PoseStamped MakePoseStamped(double x, double y, double z, double yaw)
{
    geometry_msgs::PoseStamped pt;
    pt.pose.position.x = x;
    pt.pose.position.y = y;
    pt.pose.position.z = z;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pt;
}

nav_msgs::Path point(const geometry_msgs::Pose* start_pose = nullptr)
{
    nav_msgs::Path waypoints;
    double h = 1.0;
    double scale = 7.0;

    const double base_points[][3] = {
        {scale * 2.0, scale * 0.0, h},
        {scale * 4.0, scale * 0.0, h},
        {scale * 5.0, scale * 0.25, h},
        {scale * 5.3, scale * 0.5, h},
        {scale * 5.0, scale * 0.75, h},
        {scale * 4.0, scale * 1.0, h},
        {scale * 2.0, scale * 1.0, h},
        {scale * 0.0, scale * 1.0, h},
    };

    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    if (start_pose != nullptr) {
        dx = start_pose->position.x - base_points[0][0];
        dy = start_pose->position.y - base_points[0][1];
        dz = start_pose->position.z - base_points[0][2];
    }

    for (const auto& point_xyz : base_points) {
        waypoints.poses.push_back(MakePoseStamped(
            point_xyz[0] + dx,
            point_xyz[1] + dy,
            point_xyz[2] + dz,
            0.0
        ));
    }

    return waypoints;
}

// Circle trajectory
nav_msgs::Path circle(
    int num_points = 30,
    int closure_points = 2,
    const geometry_msgs::Pose* start_pose = nullptr)
{
    double radius = 5.0;  // 圆半径
    double h = 2.0;       // 飞行高度
    double center_x = 0.0; // 圆心X坐标
    double center_y = 0.0; // 圆心Y坐标
    double center_z = h;

    if (start_pose != nullptr)
    {
        center_x = start_pose->position.x - radius;
        center_y = start_pose->position.y;
        center_z = start_pose->position.z;
    }

    nav_msgs::Path waypoints;

    // x=center_x+rcos(θ)
    // y=center_y+rsin(θ)
    // 生成圆轨迹
    for (int i = 0; i < num_points; ++i) 
    {
        double theta = 2 * M_PI * i / num_points; // 角度参数
        double x = center_x + radius * cos(theta);
        double y = center_y + radius * sin(theta);

        // 计算航向角（沿切线方向）
        double dx_dt = -radius * sin(theta); // 导数
        double dy_dt =  radius * cos(theta);
        double yaw = atan2(dy_dt, dx_dt); // 切线方向的航向角
        waypoints.poses.push_back(MakePoseStamped(x, y, center_z, yaw));
    }
    
    if (num_points > 0 && closure_points > 0 && num_points >= closure_points)
    {
        // 重复前两个点，让多项式在接缝处平滑过渡，不强制飞回圆心
        for (int i = 0; i < closure_points; ++i) {
            waypoints.poses.push_back(waypoints.poses[i]);
        }
    }

    return waypoints;
}

nav_msgs::Path eight(
    int num_points = 30,
    int closure_points = 1,
    const geometry_msgs::Pose* start_pose = nullptr)
{
    double offset_x = 0.0;  // 轨迹整体偏移
    double offset_y = 0.0;
    double r = 10.0;        // 半径
    double h = 2.0;         // 高度

    if (start_pose != nullptr)
    {
        offset_x = start_pose->position.x - r;
        offset_y = start_pose->position.y;
        h = start_pose->position.z;
    }

    nav_msgs::Path waypoints;

    // 生成一个8字循环（逆时针）
    // 8字轨迹参数方程
    // x(t)=rcos(t)
    // y(t)=rsin(2t)
    for (int i = 0; i < num_points; ++i) {
        double t = i * 2 * M_PI / num_points;  // 时间参数
        double x = r * cos(t) + offset_x;
        double y = r * sin(2 * t) + offset_y;
        double z = h;

        // 计算航向角（示例：沿轨迹切线方向）
        double dx_dt = -r * sin(t);
        double dy_dt = 2 * r * cos(2 * t);
        double yaw = atan2(dy_dt, dx_dt);
        waypoints.poses.push_back(MakePoseStamped(x, y, z, yaw));
    }
    if (num_points > 0 && closure_points > 0 && num_points >= closure_points)
    {
        // 重复前closure_points个点，让多项式在接缝处平滑过渡，不强制飞回圆心
        for (int i = 0; i < closure_points; ++i) {
            waypoints.poses.push_back(waypoints.poses[i]);
        }
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
