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
    double radius = 5.0;
    double h = 2.0;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = h;

    if (start_pose != nullptr)
    {
        center_x = start_pose->position.x - radius;
        center_y = start_pose->position.y;
        center_z = start_pose->position.z;
    }

    nav_msgs::Path waypoints;

    for (int i = 0; i < num_points; ++i) 
    {
        double theta = 2 * M_PI * i / num_points;
        double x = center_x + radius * cos(theta);
        double y = center_y + radius * sin(theta);

        double dx_dt = -radius * sin(theta);
        double dy_dt =  radius * cos(theta);
        double yaw = atan2(dy_dt, dx_dt);
        waypoints.poses.push_back(MakePoseStamped(x, y, center_z, yaw));
    }
    
    if (num_points > 0 && closure_points > 0 && num_points >= closure_points)
    {
        // Repeat leading samples so polynomial fitting remains smooth at the seam.
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
    double offset_x = 0.0;
    double offset_y = 0.0;
    double r = 10.0;
    double h = 2.0;

    if (start_pose != nullptr)
    {
        offset_x = start_pose->position.x - r;
        offset_y = start_pose->position.y;
        h = start_pose->position.z;
    }

    nav_msgs::Path waypoints;

    // x(t)=rcos(t)
    // y(t)=rsin(2t)
    for (int i = 0; i < num_points; ++i) {
        double t = i * 2 * M_PI / num_points;
        double x = r * cos(t) + offset_x;
        double y = r * sin(2 * t) + offset_y;
        double z = h;

        double dx_dt = -r * sin(t);
        double dy_dt = 2 * r * cos(2 * t);
        double yaw = atan2(dy_dt, dx_dt);
        waypoints.poses.push_back(MakePoseStamped(x, y, z, yaw));
    }
    if (num_points > 0 && closure_points > 0 && num_points >= closure_points)
    {
        // Repeat leading samples so polynomial fitting remains smooth at the seam.
        for (int i = 0; i < closure_points; ++i) {
            waypoints.poses.push_back(waypoints.poses[i]);
        }
    }
    return waypoints;
}

#endif
