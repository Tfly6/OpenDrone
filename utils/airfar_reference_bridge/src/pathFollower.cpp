#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <opendrone/PlannerOutput.h>
#include <opendrone/PlannerOutputPoint.h>

#include <airfar_reference_bridge/arrival_contract.h>
#include <airfar_reference_bridge/planner_reference.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/state_estimation";
string desiredTrajFile;
string executedTrajFile;
bool saveTrajectory = false;
double saveTrajInverval = 0.1;
bool waypointTest = false;
int waypointNum = 6;
double waypointInterval = 1.0;
double waypointYaw = 45.0;
double waypointZ = 2.0;
bool autonomyMode = false;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool trackingCamBackward = false;
double trackingCamXOffset = 0;
double trackingCamYOffset = 0;
double trackingCamZOffset = 0;
double trackingCamScale = 1.0;
double lookAheadScale = 0.2;
double minLookAheadDis = 0.2;
double minSpeed = 0.5;
double maxSpeed = 2.0;
double cruiseSpeed = -1.0;
double desiredSpeed = minSpeed;
double smoothIncrSpeed = 0.75;
double yawRateScale = 1.0;
double velZScale = 1.0;
double manualSpeedXY = 2.0;
double manualSpeedZ = 1.0;
double manualYawRate = 60.0;
double slowTurnRate = 0.75;
double minSlowTurnCurv = 0.9;
double minSlowTurnInterval = 1.0;
double minStopRotInterval = 1.0;
double stopRotDelayTime = 0;
double stopRotDis = 1.0;
double stopRotYaw1 = 90.0;
double stopRotYaw2 = 10.0;
double goalTolerance = 0.4;
double slowDis = 2.0;
double speedRatioDeadband = 0.1;
bool shiftGoalAtStart = false;
double goalX = 0;
double goalY = 0;
double goalZ = 1.0;

int trackPathID = 0;
const int stackNum = 200;
int trackPathIDStack[stackNum];
double odomTimeStack[stackNum];
float odomYawStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

bool manualMode = true;
bool autoAdjustMode = false;
int stateInitDelay = 100;

bool pathFound = false;
bool trackReferenceInitialized = false;
double stopRotTime = 0;
double slowTurnTime = 0;
double autoModeTime = 0;
int waypointCount = 0;
double waypointTime = 0;

float joyFwd = 0;
float joyFwdDb = 0;
float joyLeft = 0;
float joyUp = 0;
float joyYaw = 0;

float trackX = 0;
float trackY = 0;
float trackZ = 0;
float trackPitch = 0;
float trackYaw = 0;

float trackRecX = 0;
float trackRecY = 0;
float trackRecZ = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleYaw = 0;

float vehicleRecX = 0;
float vehicleRecY = 0;
float vehicleRecZ = 0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;

float vehicleAngRateX = 0;
float vehicleAngRateY = 0;
float vehicleAngRateZ = 0;

visualization_msgs::Marker trackMarker;
nav_msgs::Odometry trackOdom;
nav_msgs::Path trackPath, trackPath2, trackPathShow;
std_msgs::Float32 autoMode;
geometry_msgs::PointStamped waypoint;
tf::StampedTransform odomTrans;

ros::Publisher *pubMarkerPointer;
ros::Publisher *pubOdometryPointer;
ros::Publisher *pubPathPointer;
ros::Publisher *pubPlannerOutputPointer;
ros::Publisher *pubAutoModePointer;
ros::Publisher *pubWaypointPointer;
tf::TransformBroadcaster *tfBroadcasterPointer;
uint64_t plannerOutputID = 0;
airfar_reference_bridge::PlannerReferenceBuilder plannerReferenceBuilder;

FILE *desiredTrajFilePtr = NULL;
FILE *executedTrajFilePtr = NULL;

void stateEstimationHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (stateInitDelay >= 0 && shiftGoalAtStart) {
    if (stateInitDelay == 0) {
      goalX += trackingCamScale * odom->pose.pose.position.x;
      goalY += trackingCamScale * odom->pose.pose.position.y;
      goalZ += trackingCamScale * odom->pose.pose.position.z;
    }
    stateInitDelay--;
    return;
  }

  pubSkipCount--;
  if (pubSkipCount >= 0) {
    return;
  } else {
    pubSkipCount = pubSkipNum;
  }

  double odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleX = trackingCamScale * odom->pose.pose.position.x;
  vehicleY = trackingCamScale * odom->pose.pose.position.y;
  vehicleZ = trackingCamScale * odom->pose.pose.position.z;
  vehicleVelX = trackingCamScale * odom->twist.twist.linear.x;
  vehicleVelY = trackingCamScale * odom->twist.twist.linear.y;
  vehicleVelZ = trackingCamScale * odom->twist.twist.linear.z;
  vehicleAngRateX = odom->twist.twist.angular.x;
  vehicleAngRateY = odom->twist.twist.angular.y;
  vehicleAngRateZ = odom->twist.twist.angular.z;
  vehicleYaw = yaw;

  if (trackingCamBackward) {
    roll = -roll;
    pitch = -pitch;
    vehicleX = -vehicleX;
    vehicleY = -vehicleY;
    vehicleVelX = -vehicleVelX;
    vehicleVelY = -vehicleVelY;
    vehicleAngRateX = -vehicleAngRateX;
    vehicleAngRateY = -vehicleAngRateY;
  }

  float sinRoll = sin(roll);
  float cosRoll = cos(roll);
  float sinPitch = sin(pitch);
  float cosPitch = cos(pitch);
  float sinYaw = sin(yaw);
  float cosYaw = cos(yaw);

  float pointX1 = trackingCamXOffset;
  float pointY1 = trackingCamYOffset * cosRoll - trackingCamZOffset * sinRoll;
  float pointZ1 = trackingCamYOffset * sinRoll + trackingCamZOffset * cosRoll;

  float pointX2 = pointX1 * cosPitch + pointZ1 * sinPitch;
  float pointY2 = pointY1;
  float pointZ2 = -pointX1 * sinPitch + pointZ1 * cosPitch;

  vehicleX -= pointX2 * cosYaw - pointY2 * sinYaw - trackingCamXOffset;
  vehicleY -= pointX2 * sinYaw + pointY2 * cosYaw - trackingCamYOffset;
  vehicleZ -= pointZ2 - trackingCamZOffset;

  vehicleVelX -= -trackingCamYOffset * vehicleAngRateZ + trackingCamZOffset * vehicleAngRateY;
  vehicleVelY -= -trackingCamZOffset * vehicleAngRateX + trackingCamXOffset * vehicleAngRateZ;
  vehicleVelZ -= -trackingCamXOffset * vehicleAngRateY + trackingCamYOffset * vehicleAngRateX;

  float velX1 = vehicleVelX;
  float velY1 = vehicleVelY * cosRoll - vehicleVelZ * sinRoll;
  float velZ1 = vehicleVelY * sinRoll + vehicleVelZ * cosRoll;

  vehicleVelX = velX1 * cosPitch + velZ1 * sinPitch;
  vehicleVelY = velY1;
  vehicleVelZ = -velX1 * sinPitch + velZ1 * cosPitch;

  // Upstream starts its simulator at (0, 0, 1). OpenDrone starts this node
  // after takeoff, so seed the path history from the actual first odometry
  // sample instead of emitting that simulator-specific hard-coded pose.
  if (!trackReferenceInitialized) {
    trackX = vehicleX;
    trackY = vehicleY;
    trackZ = vehicleZ;
    trackYaw = vehicleYaw;
    trackPath.poses[0].pose.position.x = trackX;
    trackPath.poses[0].pose.position.y = trackY;
    trackPath.poses[0].pose.position.z = trackZ;
    trackReferenceInitialized = true;
  }

  float vehicleSpeed = sqrt(vehicleVelX * vehicleVelX + vehicleVelY * vehicleVelY);
  float referenceSpeed = 0;
  float referenceVerticalVelocity = 0;
  float referenceCurvature = 0;

  float disToGoalX = goalX - vehicleX;
  float disToGoalY = goalY - vehicleY;
  float horizontalDisToGoal = sqrt(disToGoalX * disToGoalX + disToGoalY * disToGoalY);
  float disToGoal = airfar_reference_bridge::GoalDistance3D(
      vehicleX, vehicleY, vehicleZ, goalX, goalY, goalZ);
  float dirToGoal = atan2(goalY - vehicleY, goalX - vehicleX) - vehicleYaw;
  if (dirToGoal > PI) dirToGoal -= 2 * PI;
  else if (dirToGoal < -PI) dirToGoal += 2 * PI;

  if (autonomyMode) {
    if (odomTime > stopRotTime + minStopRotInterval && horizontalDisToGoal > stopRotDis && (fabs(dirToGoal) > stopRotYaw1 * PI / 180.0 ||
        (fabs(dirToGoal) > stopRotYaw2 * PI / 180.0 && vehicleSpeed < minSpeed / 2.0)) && !autoAdjustMode) {
      joyFwd = 0;
      joyLeft = 0;
      joyUp = 0;
      joyYaw = 0;

      stopRotTime = odomTime;
      if (vehicleSpeed < minSpeed / 2.0) stopRotTime -= stopRotDelayTime;

      autoAdjustMode = true;
    }

    if (odomTime > stopRotTime + stopRotDelayTime && autoAdjustMode) {
      if (fabs(dirToGoal) > stopRotYaw2 * PI / 180.0) {
        joyFwd = 0;
        joyLeft = 0;
        joyUp = 0;
        if (dirToGoal < 0) joyYaw = -1.0;
        else joyYaw = 1.0;
      } else {
        joyFwd = 1.0;
        joyLeft = 0;
        joyUp = 0;
        joyYaw = 0;

        autoAdjustMode = false;
      }
    }

    if (odomTime - autoModeTime > 0.0667) {
      if (autoAdjustMode) autoMode.data = -1.0;
      else autoMode.data = desiredSpeed / maxSpeed;

      pubAutoModePointer->publish(autoMode);
      autoModeTime = odomTime;
    }
  } else {
    autoAdjustMode = false;
  }

  if (manualMode || (autonomyMode && autoAdjustMode)) {
    trackX = vehicleX;
    trackY = vehicleY;
    trackZ = vehicleZ;
    trackYaw = vehicleYaw;
  } else {
    trackX = trackPath.poses[trackPathID].pose.position.x;
    trackY = trackPath.poses[trackPathID].pose.position.y;
    trackZ = trackPath.poses[trackPathID].pose.position.z;

    if (joyFwdDb < joyFwd) joyFwdDb = joyFwd;
    else if (joyFwdDb > joyFwd + speedRatioDeadband) joyFwdDb = joyFwd + speedRatioDeadband;
    if (joyFwd <= speedRatioDeadband) joyFwdDb = 0;

    float joyFwd2 = joyFwdDb;
    if (joyFwd2 < minSpeed / maxSpeed) joyFwd2 = 0;

    float lookAheadDis = lookAheadScale * maxSpeed * joyFwd2;
    if (autonomyMode) lookAheadDis = lookAheadScale * desiredSpeed;
    if (lookAheadDis <= 0) lookAheadDis = 0;
    else if (lookAheadDis < minLookAheadDis) lookAheadDis = minLookAheadDis;

    float disX = trackX - vehicleX;
    float disY = trackY - vehicleY;
    float dis = sqrt(disX * disX + disY * disY);

    int trackPathLength = trackPath.poses.size();
    while (trackPathID < trackPathLength - 1) {
      float trackNextX = trackPath.poses[trackPathID + 1].pose.position.x;
      float trackNextY = trackPath.poses[trackPathID + 1].pose.position.y;
      float trackNextZ = trackPath.poses[trackPathID + 1].pose.position.z;

      float disNextX = trackNextX - vehicleX;
      float disNextY = trackNextY - vehicleY;
      float disNext = sqrt(disNextX * disNextX + disNextY * disNextY);

      if (fabs(disNext - lookAheadDis) <= fabs(dis - lookAheadDis) || disNext <= dis) {
        trackX = trackNextX;
        trackY = trackNextY;
        trackZ = trackNextZ;

        dis = disNext;
        trackPathID++;
      } else {
        break;
      }
    }

    float curv = 0;
    float slope = 0;
    if (trackPathID > 0) {
      float deltaX = trackX - trackPath.poses[trackPathID - 1].pose.position.x;
      float deltaY = trackY - trackPath.poses[trackPathID - 1].pose.position.y;
      float deltaZ = trackZ - trackPath.poses[trackPathID - 1].pose.position.z;
      trackYaw = atan2(deltaY, deltaX);

      float deltaDis = sqrt(deltaX * deltaX + deltaY * deltaY);
      if (deltaDis > 0.001 && trackPathID < trackPathLength - 1) {
        deltaX = trackPath.poses[trackPathID + 1].pose.position.x - trackPath.poses[trackPathID].pose.position.x;
        deltaY = trackPath.poses[trackPathID + 1].pose.position.y - trackPath.poses[trackPathID].pose.position.y;
        float trackNextYaw = atan2(deltaY, deltaX);

        float deltaYaw = trackNextYaw - trackYaw;
        if (deltaYaw > PI) deltaYaw -= 2 * PI;
        else if (deltaYaw < -PI) deltaYaw += 2 * PI;

        curv = deltaYaw / deltaDis;
        slope = deltaZ / deltaDis;
      }
    }

    float dirToPath = trackYaw - vehicleYaw;
    if (dirToPath > PI) dirToPath -= 2 * PI;
    else if (dirToPath < -PI) dirToPath += 2 * PI;

    float vehicleVelPath = vehicleVelX * cos(dirToPath) + vehicleVelY * sin(dirToPath);
    float desiredSpeed2 = vehicleVelPath + smoothIncrSpeed;
    float slowTurnRate2 = 1.0;
    if (fabs(curv) > minSlowTurnCurv || odomTime < slowTurnTime + minSlowTurnInterval) {
      slowTurnRate2 = slowTurnRate;
      if (fabs(curv) > minSlowTurnCurv) slowTurnTime = odomTime;
    }
    if (autonomyMode) {
      if (desiredSpeed2 > slowTurnRate2 * desiredSpeed) desiredSpeed2 = slowTurnRate2 * desiredSpeed;
    } else {
      if (desiredSpeed2 > slowTurnRate2 * maxSpeed * joyFwd2) desiredSpeed2 = slowTurnRate2 * maxSpeed * joyFwd2;
    }
    if (desiredSpeed2 < minSpeed) desiredSpeed2 = minSpeed;
    if (joyFwd2 == 0 || !pathFound || (disToGoal <= goalTolerance && autonomyMode && !autoAdjustMode)) {
      desiredSpeed2 = 0;
    } else if (disToGoal < slowDis && autonomyMode && !autoAdjustMode) {
      float slowSpeed = (maxSpeed * (disToGoal - goalTolerance) +
                         minSpeed * (slowDis - disToGoal)) /
                        (slowDis - goalTolerance);
      if (desiredSpeed2 > slowSpeed) desiredSpeed2 = slowSpeed;
    }

    referenceSpeed = desiredSpeed2;
    referenceVerticalVelocity = velZScale * slope * desiredSpeed2;
    referenceCurvature = curv;
  }

  // Arrival is a 3-D contract shared with OpenDrone's mission evaluator.
  // Once the vehicle enters that region, command the actual goal position so
  // the controller can hold it for the required dwell instead of freezing a
  // look-ahead point somewhere inside a larger horizontal stop radius.
  if (autonomyMode && !autoAdjustMode && disToGoal <= goalTolerance) {
    trackX = goalX;
    trackY = goalY;
    trackZ = goalZ;
    referenceSpeed = 0;
    referenceVerticalVelocity = 0;
    referenceCurvature = 0;
  }

  odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
  odomTimeStack[odomSendIDPointer] = odomTime;
  odomYawStack[odomSendIDPointer] = trackYaw;
  trackPathIDStack[odomSendIDPointer] = trackPathID;

  if (autonomyMode && waypointTest) {
    if (odomTime - waypointTime > waypointInterval) {
      float angle = 0, elev = 0;
      if (waypointCount > 0 && waypointCount < waypointNum - 1) {
        if (waypointCount % 2 == 0) {
          angle = -waypointYaw;
          elev = -waypointZ;
        } else {
          angle = waypointYaw;
          elev = waypointZ;
        }
      }

      if (waypointCount < waypointNum) {
        waypoint.header.stamp = odom->header.stamp;
        waypoint.header.frame_id = "/map";
        waypoint.point.x = 10.0 * cos(vehicleYaw + angle * PI / 180.0) + vehicleX;
        waypoint.point.y = 10.0 * sin(vehicleYaw + angle * PI / 180.0) + vehicleY;
        waypoint.point.z = vehicleZ + elev;
        pubWaypointPointer->publish(waypoint);

        waypointTime = odomTime;
        if (waypointCount == 1 || waypointCount == waypointNum - 2) waypointTime -= waypointInterval / 2.0;
        waypointCount++;
      }
    }
  }

  if (saveTrajectory) {
    float disX = trackX - trackRecX;
    float disY = trackY - trackRecY;
    float disZ = trackZ - trackRecZ;
    float dis = sqrt(disX * disX + disY * disY + disZ * disZ);

    float disX2 = vehicleX - vehicleRecX;
    float disY2 = vehicleY - vehicleRecY;
    float disZ2 = vehicleZ - vehicleRecZ;
    float dis2 = sqrt(disX2 * disX2 + disY2 * disY2 + disZ2 * disZ2);

    if (dis > saveTrajInverval && dis2 > saveTrajInverval) {
      fprintf(desiredTrajFilePtr, "%f %f %f %f %lf\n", trackX, trackY, trackZ, trackYaw, odomTime);
      fprintf(executedTrajFilePtr, "%f %f %f %f %f %f %lf\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, odomTime);

      trackRecX = trackX;
      trackRecY = trackY;
      trackRecZ = trackZ;

      vehicleRecX = vehicleX;
      vehicleRecY = vehicleY;
      vehicleRecZ = vehicleZ;
    }
  }

  // Preserve the upstream path selection and look-ahead decisions, then make
  // the control boundary explicit: PlannerOutput carries world-frame
  // kinematics, never the upstream actuator-level feedback commands.
  opendrone::PlannerOutput plannerOutput;
  plannerOutput.header.stamp = odom->header.stamp;
  plannerOutput.header.frame_id = "map";
  plannerOutput.trajectory_id = ++plannerOutputID;
  plannerOutput.is_horizon = false;
  plannerOutput.trajectory_start_time = odom->header.stamp;
  plannerOutput.points.resize(1);
  airfar_reference_bridge::PlannerReferenceInput referenceInput;
  if (manualMode) {
    referenceInput.mode =
        airfar_reference_bridge::ReferenceMode::kManual;
  } else if (autonomyMode && autoAdjustMode) {
    referenceInput.mode =
        airfar_reference_bridge::ReferenceMode::kRotateInPlace;
  } else {
    referenceInput.mode =
        airfar_reference_bridge::ReferenceMode::kTrackPath;
  }
  referenceInput.stamp = odomTime;
  referenceInput.vehicle_x = vehicleX;
  referenceInput.vehicle_y = vehicleY;
  referenceInput.vehicle_z = vehicleZ;
  referenceInput.vehicle_yaw = vehicleYaw;
  referenceInput.track_x = trackX;
  referenceInput.track_y = trackY;
  referenceInput.track_z = trackZ;
  referenceInput.track_yaw = trackYaw;
  referenceInput.path_speed = referenceSpeed;
  referenceInput.path_vertical_velocity = referenceVerticalVelocity;
  referenceInput.path_curvature = referenceCurvature;
  referenceInput.path_yaw_rate =
      yawRateScale * referenceCurvature * referenceSpeed;
  referenceInput.goal_x = goalX;
  referenceInput.goal_y = goalY;
  referenceInput.manual_forward_velocity = manualSpeedXY * joyFwd;
  referenceInput.manual_left_velocity = manualSpeedXY * joyLeft;
  referenceInput.manual_vertical_velocity = manualSpeedZ * joyUp;
  referenceInput.manual_yaw_rate =
      manualYawRate * joyYaw * PI / 180.0;
  plannerOutput.points.front() =
      plannerReferenceBuilder.Build(referenceInput);
  opendrone::PlannerOutputPoint& reference = plannerOutput.points.front();
  reference.time_from_start = ros::Duration(0);
  pubPlannerOutputPointer->publish(plannerOutput);

  trackMarker.header.stamp = odom->header.stamp;
  trackMarker.header.frame_id = "map";
  trackMarker.ns = "track_point";
  trackMarker.id = 0;
  trackMarker.type = visualization_msgs::Marker::SPHERE;
  trackMarker.action = visualization_msgs::Marker::ADD;
  trackMarker.scale.x = 0.2;
  trackMarker.scale.y = 0.2;
  trackMarker.scale.z = 0.2;
  trackMarker.color.a = 1.0;
  trackMarker.color.r = 1.0;
  trackMarker.pose.position.x = trackX;
  trackMarker.pose.position.y = trackY;
  trackMarker.pose.position.z = trackZ;
  pubMarkerPointer->publish(trackMarker);

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, trackPitch, trackYaw);

  trackOdom.header.stamp = odom->header.stamp;
  trackOdom.header.frame_id = "map";
  trackOdom.child_frame_id = "track_point";
  trackOdom.pose.pose.orientation = geoQuat;
  trackOdom.pose.pose.position.x = trackX;
  trackOdom.pose.pose.position.y = trackY;
  trackOdom.pose.pose.position.z = trackZ;
  trackOdom.twist.twist.angular.x = roll;
  trackOdom.twist.twist.angular.y = pitch;
  trackOdom.twist.twist.angular.z = yaw;
  trackOdom.twist.twist.linear.x = vehicleX;
  trackOdom.twist.twist.linear.y = vehicleY;
  trackOdom.twist.twist.linear.z = vehicleZ;
  pubOdometryPointer->publish(trackOdom);

  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "track_point";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(trackX, trackY, trackZ));
  tfBroadcasterPointer->sendTransform(odomTrans);
}

void pathHandler(const nav_msgs::Path::ConstPtr& path)
{
  double pathTime = path->header.stamp.toSec();

  int pathLength = path->poses.size();
  if (pathLength > 1) {
    pathFound = true;
  } else {
    pathLength = 1;
    pathFound = false;
  }

  if (odomSendIDPointer < 0) {
    return;
  }

  while (odomRecIDPointer != (odomSendIDPointer + 1) % stackNum) {
    int odomRecIDPointerNext = (odomRecIDPointer + 1) % stackNum;
    if (fabs(pathTime - odomTimeStack[odomRecIDPointer]) < fabs(pathTime - odomTimeStack[odomRecIDPointerNext])) {
      break;
    }
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  int trackPathRecID = trackPathIDStack[odomRecIDPointer];
  if (trackPathRecID < 100) {
    trackPath2 = trackPath;
    trackPath.poses.resize(trackPathRecID + pathLength);
    for (int i = 0; i <= trackPathRecID; i++) {
      trackPath.poses[i] = trackPath2.poses[i];
    }
  } else {
    trackPath2.poses.resize(101);
    for (int i = 0; i <= 100; i++) {
      trackPath2.poses[i] = trackPath.poses[trackPathRecID + i - 100];
    }
    trackPath.poses.resize(trackPathRecID + pathLength);
    for (int i = 0; i <= 100; i++) {
      trackPath.poses[trackPathRecID + i - 100] = trackPath2.poses[i];
    }
  }

  if (manualMode || (autonomyMode && autoAdjustMode)) {
    trackPath.poses[trackPathRecID].pose.position.x = trackX;
    trackPath.poses[trackPathRecID].pose.position.y = trackY;
    trackPath.poses[trackPathRecID].pose.position.z = trackZ;
    if (trackPathRecID > 0) {
      trackPath.poses[trackPathRecID - 1].pose.position.x = trackX - 0.1 * cos(trackYaw);
      trackPath.poses[trackPathRecID - 1].pose.position.y = trackY - 0.1 * sin(trackYaw);
      trackPath.poses[trackPathRecID - 1].pose.position.z = trackZ;
    }
    odomYawStack[odomRecIDPointer] = trackYaw;
  }

  trackX = trackPath.poses[trackPathRecID].pose.position.x;
  trackY = trackPath.poses[trackPathRecID].pose.position.y;
  trackZ = trackPath.poses[trackPathRecID].pose.position.z;
  trackYaw = odomYawStack[odomRecIDPointer];

  float sinTrackPitch = sin(trackPitch);
  float cosTrackPitch = cos(trackPitch);
  float sinTrackYaw = sin(trackYaw);
  float cosTrackYaw = cos(trackYaw);

  for (int i = 1; i < pathLength; i++) {
    float trackX2 = cosTrackPitch * path->poses[i].pose.position.x + sinTrackPitch * path->poses[i].pose.position.z;
    float trackY2 = path->poses[i].pose.position.y;
    float trackZ2 = -sinTrackPitch * path->poses[i].pose.position.x + cosTrackPitch * path->poses[i].pose.position.z;

    trackPath.poses[trackPathRecID + i].pose.position.x = cosTrackYaw * trackX2 - sinTrackYaw * trackY2 + trackX;
    trackPath.poses[trackPathRecID + i].pose.position.y = sinTrackYaw * trackX2 + cosTrackYaw * trackY2 + trackY;
    trackPath.poses[trackPathRecID + i].pose.position.z = trackZ2 + trackZ;
  }

  int trackPathLength = trackPath.poses.size();
  if (trackPathLength < 500) {
    trackPathShow = trackPath;
  } else {
    trackPathShow.poses.resize(500);
    for (int i = 0; i < 500; i++) {
      trackPathShow.poses[i] = trackPath.poses[trackPathLength + i - 500];
    }
  }

  trackPathShow.header.stamp = path->header.stamp;
  trackPathShow.header.frame_id = "map";
  pubPathPointer->publish(trackPathShow);
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalZ = goal->point.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("desiredTrajFile", desiredTrajFile);
  nhPrivate.getParam("executedTrajFile", executedTrajFile);
  nhPrivate.getParam("saveTrajectory", saveTrajectory);
  nhPrivate.getParam("saveTrajInverval", saveTrajInverval);
  nhPrivate.getParam("waypointTest", waypointTest);
  nhPrivate.getParam("waypointNum", waypointNum);
  nhPrivate.getParam("waypointInterval", waypointInterval);
  nhPrivate.getParam("waypointYaw", waypointYaw);
  nhPrivate.getParam("waypointZ", waypointZ);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("trackingCamBackward", trackingCamBackward);
  nhPrivate.getParam("trackingCamXOffset", trackingCamXOffset);
  nhPrivate.getParam("trackingCamYOffset", trackingCamYOffset);
  nhPrivate.getParam("trackingCamZOffset", trackingCamZOffset);
  nhPrivate.getParam("trackingCamScale", trackingCamScale);
  nhPrivate.getParam("trackPitch", trackPitch);
  nhPrivate.getParam("lookAheadScale", lookAheadScale);
  nhPrivate.getParam("minLookAheadDis", minLookAheadDis);
  nhPrivate.getParam("minSpeed", minSpeed);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("cruiseSpeed", cruiseSpeed);
  nhPrivate.getParam("smoothIncrSpeed", smoothIncrSpeed);
  nhPrivate.getParam("yawRateScale", yawRateScale);
  nhPrivate.getParam("velZScale", velZScale);
  nhPrivate.getParam("manualSpeedXY", manualSpeedXY);
  nhPrivate.getParam("manualSpeedZ", manualSpeedZ);
  nhPrivate.getParam("manualYawRate", manualYawRate);
  nhPrivate.getParam("slowTurnRate", slowTurnRate);
  nhPrivate.getParam("minSlowTurnCurv", minSlowTurnCurv);
  nhPrivate.getParam("minSlowTurnInterval", minSlowTurnInterval);
  nhPrivate.getParam("minStopRotInterval", minStopRotInterval);
  nhPrivate.getParam("stopRotDelayTime", stopRotDelayTime);
  nhPrivate.getParam("stopRotDis", stopRotDis);
  nhPrivate.getParam("stopRotYaw1", stopRotYaw1);
  nhPrivate.getParam("stopRotYaw2", stopRotYaw2);
  nhPrivate.getParam("goalTolerance", goalTolerance);
  nhPrivate.getParam("slowDis", slowDis);
  nhPrivate.getParam("speedRatioDeadband", speedRatioDeadband);
  nhPrivate.getParam("shiftGoalAtStart", shiftGoalAtStart);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);
  nhPrivate.getParam("goalZ", goalZ);

  if (cruiseSpeed < 0.0) cruiseSpeed = minSpeed;
  if (!(minSpeed > 0.0 && minSpeed <= cruiseSpeed &&
        cruiseSpeed <= maxSpeed)) {
    ROS_FATAL_STREAM("Invalid speed configuration: minSpeed=" << minSpeed
                     << ", cruiseSpeed=" << cruiseSpeed
                     << ", maxSpeed=" << maxSpeed
                     << ". Require 0 < minSpeed <= cruiseSpeed <= maxSpeed.");
    return 1;
  }
  if (!(goalTolerance > 0.0 && goalTolerance < slowDis)) {
    ROS_FATAL_STREAM("Invalid arrival configuration: goalTolerance="
                     << goalTolerance << ", slowDis=" << slowDis
                     << ". Require 0 < goalTolerance < slowDis.");
    return 1;
  }
  desiredSpeed = cruiseSpeed;
  ROS_INFO_STREAM("Air-FAR speed contract: min=" << minSpeed
                  << " m/s, cruise=" << cruiseSpeed
                  << " m/s, max=" << maxSpeed << " m/s");
  if (autonomyMode) {
    manualMode = false;
    joyFwd = 1.0;
  }

  trackPath.poses.resize(1);
  trackPath.poses[0].pose.position.x = 0;
  trackPath.poses[0].pose.position.y = 0;
  trackPath.poses[0].pose.position.z = 0;

  if (saveTrajectory) {
    desiredTrajFilePtr = fopen(desiredTrajFile.c_str(), "w");
    executedTrajFilePtr = fopen(executedTrajFile.c_str(), "w");
  }

  ros::Subscriber subStateEstimation = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, stateEstimationHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/path", 5, pathHandler);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  ros::Publisher pubMarker = nh.advertise<visualization_msgs::Marker> ("/track_point_marker", 5);
  pubMarkerPointer = &pubMarker;

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/track_point_odom", 5);
  pubOdometryPointer = &pubOdometry;

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/track_path", 5);
  pubPathPointer = &pubPath;

  ros::Publisher pubPlannerOutput =
      nh.advertise<opendrone::PlannerOutput> ("/planner/output", 5);
  pubPlannerOutputPointer = &pubPlannerOutput;

  ros::Publisher pubAutoMode = nh.advertise<std_msgs::Float32> ("/auto_mode", 5);
  pubAutoModePointer = &pubAutoMode;

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
  pubWaypointPointer = &pubWaypoint;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::spin();

  if (saveTrajectory) {
    fclose(desiredTrajFilePtr);
    fclose(executedTrajFilePtr);

    printf("\nTrajectories saved.\n\n");
  }

  return 0;
}
