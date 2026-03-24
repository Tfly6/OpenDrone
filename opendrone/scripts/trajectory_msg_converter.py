#!/usr/bin/env python3

"""@trajectory_msg_converter.py
This node converts Fast-Planner/EGO-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by MPC controller.
Supports two modes:
  - bspline mode: subscribes to traj_utils/Bspline, samples future horizon points exactly from the B-spline
  - pos_cmd mode: subscribes to quadrotor_msgs/PositionCommand (legacy, single-point or kinematic extrapolation)
Authors: Mohamed Abdelkader
"""

import rospy
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler


class UniformBsplineEval:
    """Minimal De Boor evaluation for uniform B-spline position/velocity/acceleration."""

    def __init__(self, control_pts, order, knots):
        # control_pts: np.ndarray shape (N, 3)
        # knots: np.ndarray shape (M,)
        self.pts = control_pts
        self.order = order
        self.knots = knots

    def _de_boor(self, t):
        """Evaluate position at parameter t using De Boor's algorithm."""
        k = self.order
        knots = self.knots
        pts = self.pts
        n = len(pts) - 1

        # clamp t
        t = np.clip(t, knots[k], knots[n + 1])

        # find knot span
        idx = k
        for i in range(k, n + 1):
            if knots[i] <= t < knots[i + 1]:
                idx = i
                break
        else:
            idx = n  # clamp to last valid span

        d = [pts[j].copy() for j in range(idx - k, idx + 1)]
        for r in range(1, k + 1):
            for j in range(k, r - 1, -1):
                left = knots[idx - k + j]
                right = knots[idx + 1 + j - r]
                if abs(right - left) < 1e-10:
                    alpha = 0.0
                else:
                    alpha = (t - left) / (right - left)
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
        return d[k]

    def get_time_range(self):
        k = self.order
        n = len(self.pts) - 1
        return self.knots[k], self.knots[n + 1]

    def evaluate(self, t):
        return self._de_boor(t)

    def derivative(self):
        """Return a new UniformBsplineEval representing the derivative."""
        k = self.order
        pts = self.pts
        knots = self.knots
        n = len(pts) - 1
        new_pts = []
        for i in range(n):
            denom = knots[i + k + 1] - knots[i + 1]
            if abs(denom) < 1e-10:
                new_pts.append(np.zeros(3))
            else:
                new_pts.append(k * (pts[i + 1] - pts[i]) / denom)
        new_knots = knots[1:-1]
        return UniformBsplineEval(np.array(new_pts), k - 1, new_knots)


class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        rospy.logwarn("---------------OK!")

        planner_traj_topic = rospy.get_param('~planner_traj_topic', 'planning/pos_cmd')
        planner_bspline_topic = rospy.get_param('~planner_bspline_topic', '')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'command/trajectory')
        self.publish_horizon_points = int(rospy.get_param('~publish_horizon_points', 60))
        self.publish_dt = float(rospy.get_param('~publish_dt', 0.02))

        if self.publish_horizon_points < 1:
            self.publish_horizon_points = 1
        if self.publish_dt <= 0.0:
            self.publish_dt = 0.02

        # B-spline state (set by BsplineCallback)
        self._bspline_pos = None
        self._bspline_vel = None
        self._bspline_acc = None
        self._bspline_start_time = None
        self._bspline_t0 = 0.0
        self._bspline_t1 = 0.0
        self._last_yaw = 0.0

        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)

        self._use_bspline = bool(planner_bspline_topic)
        if self._use_bspline:
            rospy.Subscriber(planner_bspline_topic, Bspline, self.BsplineCallback, tcp_nodelay=True)
            rospy.loginfo("trajectory_msg_converter: bspline mode, topic=%s, horizon=%d, dt=%.3f",
                          planner_bspline_topic, self.publish_horizon_points, self.publish_dt)
            # publish at fixed rate from stored bspline
            rospy.Timer(rospy.Duration(self.publish_dt), self._bspline_publish_timer)
        else:
            rospy.Subscriber(planner_traj_topic, PositionCommand, self.PlannerTrajCallback, tcp_nodelay=True)
            rospy.loginfo(
                "trajectory_msg_converter: pos_cmd mode, topic=%s, horizon_points=%d, dt=%.3f",
                planner_traj_topic, self.publish_horizon_points, self.publish_dt
            )

        rospy.spin()

    def BsplineCallback(self, msg):
        pts = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
        knots = np.array(msg.knots)
        pos_spline = UniformBsplineEval(pts, msg.order, knots)
        vel_spline = pos_spline.derivative()
        acc_spline = vel_spline.derivative()
        t0, t1 = pos_spline.get_time_range()
        self._bspline_pos = pos_spline
        self._bspline_vel = vel_spline
        self._bspline_acc = acc_spline
        self._bspline_start_time = msg.start_time
        self._bspline_t0 = t0
        self._bspline_t1 = t1

    def _bspline_publish_timer(self, event):
        if self._bspline_pos is None:
            return
        time_now = rospy.Time.now()
        t_cur = (time_now - self._bspline_start_time).to_sec() + self._bspline_t0
        if t_cur > self._bspline_t1:
            return  # trajectory finished

        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = time_now
        traj_msg.header.frame_id = 'world'

        for i in range(self.publish_horizon_points):
            t = np.clip(t_cur + i * self.publish_dt, self._bspline_t0, self._bspline_t1)

            pos = self._bspline_pos.evaluate(t)
            vel = self._bspline_vel.evaluate(t)
            acc = self._bspline_acc.evaluate(t)

            # yaw: point toward velocity direction
            yaw = self._last_yaw
            if np.linalg.norm(vel[:2]) > 0.1:
                yaw = np.arctan2(vel[1], vel[0])
            if i == 0:
                self._last_yaw = yaw

            q = quaternion_from_euler(0, 0, yaw)
            pose = Transform()
            pose.translation.x = pos[0]
            pose.translation.y = pos[1]
            pose.translation.z = pos[2]
            pose.rotation.x = q[0]
            pose.rotation.y = q[1]
            pose.rotation.z = q[2]
            pose.rotation.w = q[3]

            vel_twist = Twist()
            vel_twist.linear.x = vel[0]
            vel_twist.linear.y = vel[1]
            vel_twist.linear.z = vel[2]

            acc_twist = Twist()
            acc_twist.linear.x = acc[0]
            acc_twist.linear.y = acc[1]
            acc_twist.linear.z = acc[2]

            traj_point = MultiDOFJointTrajectoryPoint()
            traj_point.transforms.append(pose)
            traj_point.velocities.append(vel_twist)
            traj_point.accelerations.append(acc_twist)
            traj_point.time_from_start = rospy.Duration.from_sec(i * self.publish_dt)

            traj_msg.points.append(traj_point)

        self.traj_pub.publish(traj_msg)

    def PlannerTrajCallback(self, msg):
        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        # TODO: set vel.angular to msg.yaw_dot

        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)

        self.traj_pub.publish(traj_msg)
        # rospy.logwarn("Publishing OK!")

if __name__ == '__main__':
    obj = MessageConverter()