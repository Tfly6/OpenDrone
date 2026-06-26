#include <iostream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <multi_map_server/Map2D.h>
#include <multi_map_server/Map3D.h>
#include <multi_map_server/MultiOccupancyGrid.h>
#include <multi_map_server/MultiSparseMap3D.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "math_utils/math_utils.h"

ros::Publisher pub1;
ros::Publisher pub2;

vector<Map2D> maps2d;
vector<geometry_msgs::Pose> origins2d;
vector<Map3D> maps3d;
vector<geometry_msgs::Pose> origins3d;

namespace {

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::Quaternion &q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

}  // namespace

void maps2d_callback(const multi_map_server::MultiOccupancyGrid::ConstPtr &msg)
{
  maps2d.resize(msg->maps.size(), Map2D(4));
  for (unsigned int k = 0; k < msg->maps.size(); k++)
    maps2d[k].Replace(msg->maps[k]);
  origins2d = msg->origins;

  multi_map_server::MultiOccupancyGrid m;
  m.maps.resize(maps2d.size());
  m.origins.resize(maps2d.size());
  for (unsigned int k = 0; k < maps2d.size(); k++)
  {
    m.maps[k] = maps2d[k].GetMap();
    m.origins[k] = origins2d[k];
  }
  pub1.publish(m);
}

void maps3d_callback(const multi_map_server::MultiSparseMap3D::ConstPtr &msg)
{
  maps3d.resize(msg->maps.size());
  for (unsigned int k = 0; k < msg->maps.size(); k++)
    maps3d[k].UnpackMsg(msg->maps[k]);
  origins3d = msg->origins;

  sensor_msgs::PointCloud m;
  for (unsigned int k = 0; k < msg->maps.size(); k++)
  {
    Eigen::Vector3d origin(
        origins3d[k].position.x,
        origins3d[k].position.y,
        origins3d[k].position.z);
    Eigen::Vector3d ypr = R_to_ypr(quaternion_to_R(toEigenQuaternion(origins3d[k].orientation)));
    Eigen::Matrix3d rotation = ypr_to_R(ypr);
    vector<Eigen::Vector3d> pts = maps3d[k].GetOccupancyWorldFrame(OCCUPIED);
    for (unsigned int i = 0; i < pts.size(); i++)
    {
      Eigen::Vector3d pt = rotation * pts[i] + origin;
      geometry_msgs::Point32 ros_pt;
      ros_pt.x = pt.x();
      ros_pt.y = pt.y();
      ros_pt.z = pt.z();
      m.points.push_back(ros_pt);
    }
  }

  m.header.stamp = ros::Time::now();
  m.header.frame_id = string("/map");
  pub2.publish(m);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_map_visualization");
  ros::NodeHandle n("~");

  ros::Subscriber sub1 = n.subscribe("dmaps2d", 1, maps2d_callback);
  ros::Subscriber sub2 = n.subscribe("dmaps3d", 1, maps3d_callback);
  pub1 = n.advertise<multi_map_server::MultiOccupancyGrid>("maps2d", 1, true);
  pub2 = n.advertise<sensor_msgs::PointCloud>("map3d", 1, true);

  ros::spin();
  return 0;
}
