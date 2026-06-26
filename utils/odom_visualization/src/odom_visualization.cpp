#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "math_utils/math_utils.h"

using namespace std;

static string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale, rotate_yaw;

bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;
Vector6d poseOrigin = Vector6d::Zero();
ros::Publisher posePub;
ros::Publisher pathPub;
ros::Publisher velPub;
ros::Publisher covPub;
ros::Publisher covVelPub;
ros::Publisher trajPub;
ros::Publisher sensorPub;
ros::Publisher meshPub;
ros::Publisher heightPub;
tf::TransformBroadcaster *broadcaster;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path pathROS;
visualization_msgs::Marker velROS;
visualization_msgs::Marker covROS;
visualization_msgs::Marker covVelROS;
visualization_msgs::Marker trajROS;
visualization_msgs::Marker sensorROS;
visualization_msgs::Marker meshROS;
sensor_msgs::Range heightROS;
string _frame_id;
int _drone_id;

namespace {

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::Quaternion &q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

geometry_msgs::Quaternion toGeometryQuaternion(const Eigen::Quaterniond &q)
{
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

Vector6d odomToPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  Vector6d pose;
  pose << msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->pose.pose.position.z,
          R_to_ypr(quaternion_to_R(toEigenQuaternion(msg->pose.pose.orientation)));
  return pose;
}

Eigen::Vector3d odomToVelocity(const nav_msgs::Odometry::ConstPtr &msg)
{
  return Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
}

Eigen::Matrix3d makeRightHanded(Eigen::Matrix3d basis)
{
  if (basis.determinant() < 0.0) {
    basis.col(0) *= -1.0;
  }
  return basis;
}

Eigen::Quaterniond yawPitchRollQuaternion(const Eigen::Vector3d &ypr)
{
  return R_to_quaternion(ypr_to_R(ypr));
}

}  // namespace

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (msg->header.frame_id == string("null"))
    return;

  Vector6d pose = odomToPose(msg);
  Eigen::Vector3d vel = odomToVelocity(msg);

  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin = pose;
  }
  if (origin)
  {
    vel = ypr_to_R(pose.tail<3>()).transpose() * vel;
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel = ypr_to_R(pose.tail<3>()) * vel;
  }

  poseROS.header = msg->header;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  poseROS.pose.orientation = toGeometryQuaternion(yawPitchRollQuaternion(pose.tail<3>()));
  posePub.publish(poseROS);

  Eigen::Vector3d yprVel;
  yprVel << atan2(vel.y(), vel.x()),
            -atan2(vel.z(), vel.head<2>().norm()),
            0.0;
  const Eigen::Quaterniond velQuat = yawPitchRollQuaternion(yprVel);
  velROS.header.frame_id = string("world");
  velROS.header.stamp = msg->header.stamp;
  velROS.ns = string("velocity");
  velROS.id = _drone_id;
  velROS.type = visualization_msgs::Marker::ARROW;
  velROS.action = visualization_msgs::Marker::ADD;
  velROS.pose.position.x = pose(0);
  velROS.pose.position.y = pose(1);
  velROS.pose.position.z = pose(2);
  velROS.pose.orientation = toGeometryQuaternion(velQuat);
  velROS.scale.x = vel.norm();
  velROS.scale.y = 0.05;
  velROS.scale.z = 0.05;
  velROS.color.a = 1.0;
  velROS.color.r = color_r;
  velROS.color.g = color_g;
  velROS.color.b = color_b;
  velPub.publish(velROS);

  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub.publish(pathROS);
  }

  double r = 1.0;
  double g = 1.0;
  double b = 1.0;
  const bool G = msg->twist.covariance[33];
  const bool V = msg->twist.covariance[34];
  const bool L = msg->twist.covariance[35];
  if (cov_color)
  {
    r = G;
    g = V;
    b = L;
  }

  if (cov_pos)
  {
    Eigen::Matrix3d P;
    for (int j = 0; j < 3; ++j)
      for (int i = 0; i < 3; ++i)
        P(i, j) = msg->pose.covariance[i + j * 6];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P);
    Eigen::Vector3d eigVal = solver.eigenvalues().cwiseMax(0.0);
    Eigen::Matrix3d eigVec = makeRightHanded(solver.eigenvectors());

    covROS.header.frame_id = string("world");
    covROS.header.stamp = msg->header.stamp;
    covROS.ns = string("covariance");
    covROS.id = _drone_id;
    covROS.type = visualization_msgs::Marker::SPHERE;
    covROS.action = visualization_msgs::Marker::ADD;
    covROS.pose.position.x = pose(0);
    covROS.pose.position.y = pose(1);
    covROS.pose.position.z = pose(2);
    covROS.pose.orientation = toGeometryQuaternion(R_to_quaternion(eigVec));
    covROS.scale.x = sqrt(eigVal(0)) * cov_scale;
    covROS.scale.y = sqrt(eigVal(1)) * cov_scale;
    covROS.scale.z = sqrt(eigVal(2)) * cov_scale;
    covROS.color.a = 0.4;
    covROS.color.r = r * 0.5;
    covROS.color.g = g * 0.5;
    covROS.color.b = b * 0.5;
    covPub.publish(covROS);
  }

  if (cov_vel)
  {
    Eigen::Matrix3d P;
    for (int j = 0; j < 3; ++j)
      for (int i = 0; i < 3; ++i)
        P(i, j) = msg->twist.covariance[i + j * 6];
    P = ypr_to_R(pose.tail<3>()) * P * ypr_to_R(pose.tail<3>()).transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P);
    Eigen::Vector3d eigVal = solver.eigenvalues().cwiseMax(0.0);
    Eigen::Matrix3d eigVec = makeRightHanded(solver.eigenvectors());

    covVelROS.header.frame_id = string("world");
    covVelROS.header.stamp = msg->header.stamp;
    covVelROS.ns = string("covariance_velocity");
    covVelROS.id = _drone_id;
    covVelROS.type = visualization_msgs::Marker::SPHERE;
    covVelROS.action = visualization_msgs::Marker::ADD;
    covVelROS.pose.position.x = pose(0);
    covVelROS.pose.position.y = pose(1);
    covVelROS.pose.position.z = pose(2);
    covVelROS.pose.orientation = toGeometryQuaternion(R_to_quaternion(eigVec));
    covVelROS.scale.x = sqrt(eigVal(0)) * cov_scale;
    covVelROS.scale.y = sqrt(eigVal(1)) * cov_scale;
    covVelROS.scale.z = sqrt(eigVal(2)) * cov_scale;
    covVelROS.color.a = 0.4;
    covVelROS.color.r = r;
    covVelROS.color.g = g;
    covVelROS.color.b = b;
    covVelPub.publish(covVelROS);
  }

  static Eigen::Vector3d ppose = pose.head<3>();
  static ros::Time pt = msg->header.stamp;
  const ros::Time t = msg->header.stamp;
  if ((t - pt).toSec() > 0.5)
  {
    trajROS.header.frame_id = string("world");
    trajROS.header.stamp = ros::Time::now();
    trajROS.ns = string("trajectory");
    trajROS.type = visualization_msgs::Marker::LINE_LIST;
    trajROS.action = visualization_msgs::Marker::ADD;
    trajROS.pose.position.x = 0;
    trajROS.pose.position.y = 0;
    trajROS.pose.position.z = 0;
    trajROS.pose.orientation.w = 1;
    trajROS.pose.orientation.x = 0;
    trajROS.pose.orientation.y = 0;
    trajROS.pose.orientation.z = 0;
    trajROS.scale.x = 0.1;
    trajROS.scale.y = 0;
    trajROS.scale.z = 0;
    trajROS.color.r = 0.0;
    trajROS.color.g = 1.0;
    trajROS.color.b = 0.0;
    trajROS.color.a = 0.8;

    geometry_msgs::Point p;
    p.x = ppose.x();
    p.y = ppose.y();
    p.z = ppose.z();
    trajROS.points.push_back(p);
    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    trajROS.points.push_back(p);

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    trajROS.colors.push_back(color);
    trajROS.colors.push_back(color);
    ppose = pose.head<3>();
    pt = t;
    trajPub.publish(trajROS);
  }

  const Eigen::Quaterniond poseQuat = yawPitchRollQuaternion(pose.tail<3>());
  sensorROS.header.frame_id = string("world");
  sensorROS.header.stamp = msg->header.stamp;
  sensorROS.ns = string("sensor");
  sensorROS.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  sensorROS.action = visualization_msgs::Marker::ADD;
  sensorROS.pose.position.x = pose(0);
  sensorROS.pose.position.y = pose(1);
  sensorROS.pose.position.z = pose(2) + 1.0;
  sensorROS.pose.orientation = toGeometryQuaternion(poseQuat);
  string strG = G ? string(" GPS ") : string("");
  string strV = V ? string(" Vision ") : string("");
  string strL = L ? string(" Laser ") : string("");
  sensorROS.text = "| " + strG + strV + strL + " |";
  sensorROS.color.a = 1.0;
  sensorROS.color.r = 1.0;
  sensorROS.color.g = 1.0;
  sensorROS.color.b = 1.0;
  sensorROS.scale.z = 0.5;
  sensorPub.publish(sensorROS);

  const double H = msg->twist.covariance[32];
  heightROS.header.frame_id = string("height");
  heightROS.header.stamp = msg->header.stamp;
  heightROS.radiation_type = sensor_msgs::Range::ULTRASOUND;
  heightROS.field_of_view = 5.0 * M_PI / 180.0;
  heightROS.min_range = -100;
  heightROS.max_range = 100;
  heightROS.range = H;
  heightPub.publish(heightROS);

  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "drone";
  meshROS.id = _drone_id;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x;
  meshROS.pose.position.y = msg->pose.pose.position.y;
  meshROS.pose.position.z = msg->pose.pose.position.z;

  Eigen::Vector3d meshYpr = R_to_ypr(quaternion_to_R(toEigenQuaternion(msg->pose.pose.orientation)));
  meshYpr(0) += rotate_yaw * M_PI / 180.0;
  if (cross_config)
  {
    meshYpr(0) += 45.0 * M_PI / 180.0;
  }
  const Eigen::Quaterniond meshQuat = yawPitchRollQuaternion(meshYpr);
  meshROS.pose.orientation = toGeometryQuaternion(meshQuat);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);

  if (tf45)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    transform.setRotation(tf::Quaternion(
        poseQuat.x(), poseQuat.y(), poseQuat.z(), poseQuat.w()));

    tf::Transform transform45;
    transform45.setOrigin(tf::Vector3(0, 0, 0));
    Eigen::Vector3d y45 = Eigen::Vector3d::Zero();
    y45(0) = 45.0 * M_PI / 180.0;
    const Eigen::Quaterniond q45 = yawPitchRollQuaternion(y45);
    transform45.setRotation(tf::Quaternion(q45.x(), q45.y(), q45.z(), q45.w()));

    tf::Transform transform90;
    transform90.setOrigin(tf::Vector3(0, 0, 0));
    Eigen::Vector3d p90 = Eigen::Vector3d::Zero();
    p90(1) = 90.0 * M_PI / 180.0;
    const Eigen::Quaterniond q90 = yawPitchRollQuaternion(p90);
    transform90.setRotation(tf::Quaternion(q90.x(), q90.y(), q90.z(), q90.w()));

    string base_s = _drone_id == -1 ? string("base") : string("base") + std::to_string(_drone_id);
    string laser_s = _drone_id == -1 ? string("laser") : string("laser") + std::to_string(_drone_id);
    string vision_s = _drone_id == -1 ? string("vision") : string("vision") + std::to_string(_drone_id);
    string height_s = _drone_id == -1 ? string("height") : string("height") + std::to_string(_drone_id);

    broadcaster->sendTransform(tf::StampedTransform(transform, msg->header.stamp, string("world"), base_s));
    broadcaster->sendTransform(tf::StampedTransform(transform45, msg->header.stamp, base_s, laser_s));
    broadcaster->sendTransform(tf::StampedTransform(transform45, msg->header.stamp, base_s, vision_s));
    broadcaster->sendTransform(tf::StampedTransform(transform90, msg->header.stamp, base_s, height_s));
  }
}

void cmd_callback(const quadrotor_msgs::PositionCommand cmd)
{
  if (cmd.header.frame_id == string("null"))
    return;

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  if (cross_config)
  {
    Eigen::Vector3d ypr = Eigen::Vector3d::Zero();
    ypr(0) += 45.0 * M_PI / 180.0;
    q = yawPitchRollQuaternion(ypr);
  }

  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = cmd.header.stamp;
  meshROS.ns = "drone";
  meshROS.id = _drone_id;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = cmd.position.x;
  meshROS.pose.position.y = cmd.position.y;
  meshROS.pose.position.z = cmd.position.z;
  meshROS.pose.orientation = toGeometryQuaternion(q);
  meshROS.scale.x = 2.0;
  meshROS.scale.y = 2.0;
  meshROS.scale.z = 2.0;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle n("~");

  n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/fake_drone.dae"));
  n.param("color/r", color_r, 1.0);
  n.param("color/g", color_g, 0.0);
  n.param("color/b", color_b, 0.0);
  n.param("color/a", color_a, 1.0);
  n.param("origin", origin, false);
  n.param("robot_scale", scale, 2.0);
  n.param("frame_id", _frame_id, string("world"));
  n.param("rotate_yaw_deg", rotate_yaw, 0.0);

  n.param("cross_config", cross_config, false);
  n.param("tf45", tf45, false);
  n.param("covariance_scale", cov_scale, 100.0);
  n.param("covariance_position", cov_pos, false);
  n.param("covariance_velocity", cov_vel, false);
  n.param("covariance_color", cov_color, false);
  n.param("drone_id", _drone_id, -1);

  ros::Subscriber sub_odom = n.subscribe("odom", 100, odom_callback);
  ros::Subscriber sub_cmd = n.subscribe("cmd", 100, cmd_callback);
  posePub = n.advertise<geometry_msgs::PoseStamped>("pose", 100, true);
  pathPub = n.advertise<nav_msgs::Path>("path", 100, true);
  velPub = n.advertise<visualization_msgs::Marker>("velocity", 100, true);
  covPub = n.advertise<visualization_msgs::Marker>("covariance", 100, true);
  covVelPub = n.advertise<visualization_msgs::Marker>("covariance_velocity", 100, true);
  trajPub = n.advertise<visualization_msgs::Marker>("trajectory", 100, true);
  sensorPub = n.advertise<visualization_msgs::Marker>("sensor", 100, true);
  meshPub = n.advertise<visualization_msgs::Marker>("robot", 100, true);
  heightPub = n.advertise<sensor_msgs::Range>("height", 100, true);
  tf::TransformBroadcaster b;
  broadcaster = &b;

  ros::spin();

  return 0;
}
