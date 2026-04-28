#!/usr/bin/env python3
"""Converter for Air-FAR PointStamped waypoint output to MultiDOFJointTrajectory."""

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler


class WaypointConverter:
    def __init__(self):
        rospy.init_node('waypoint_msg_converter')

        waypoint_topic = rospy.get_param('~waypoint_topic', '/way_point')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', '/command/trajectory')

        self.waypoint_sub = rospy.Subscriber(waypoint_topic, PointStamped, self.callback)
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)

        self.last_yaw = 0.0
        self.last_position = None

        rospy.loginfo("waypoint_msg_converter: subscribing to %s, publishing to %s",
                      waypoint_topic, traj_pub_topic)

        rospy.spin()

    def callback(self, msg):
        pos = msg.point

        # Estimate yaw from position change
        yaw = self.last_yaw
        if self.last_position is not None:
            dx = pos.x - self.last_position[0]
            dy = pos.y - self.last_position[1]
            if np.sqrt(dx*dx + dy*dy) > 0.01:
                yaw = np.arctan2(dy, dx)

        self.last_yaw = yaw
        self.last_position = [pos.x, pos.y, pos.z]

        q = quaternion_from_euler(0, 0, yaw)

        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = 'map'

        transform = Transform()
        transform.translation.x = pos.x
        transform.translation.y = pos.y
        transform.translation.z = pos.z
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]

        point = MultiDOFJointTrajectoryPoint()
        point.transforms.append(transform)
        point.velocities.append(Twist())
        point.accelerations.append(Twist())
        point.time_from_start = rospy.Duration(0)

        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)


if __name__ == '__main__':
    WaypointConverter()