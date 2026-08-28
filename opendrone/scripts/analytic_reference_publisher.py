#!/usr/bin/env python3

"""Publish analytic reference trajectories as opendrone/PlannerOutput."""

import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from opendrone.msg import PlannerOutput, PlannerOutputPoint


class AnalyticReferencePublisher:
    def __init__(self):
        rospy.init_node('analytic_reference_publisher')

        self.output_topic = rospy.get_param('~planner_output_topic', '/planner/output')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.trajectory_type = rospy.get_param('~trajectory_type', 'circle').strip().lower()
        self.publish_rate = max(1.0, float(rospy.get_param('~publish_rate', 50.0)))
        self.sample_dt = max(0.01, float(rospy.get_param('~sample_dt', 0.05)))
        self.horizon_duration = max(self.sample_dt, float(rospy.get_param('~horizon_duration', 2.0)))
        self.radius = float(rospy.get_param('~radius', 5.0))
        self.linear_speed = max(0.01, float(rospy.get_param('~linear_speed', 1.0)))
        self.center_x = float(rospy.get_param('~center_x', 0.0))
        self.center_y = float(rospy.get_param('~center_y', 0.0))
        self.center_z = float(rospy.get_param('~center_z', 2.0))
        self.start_phase = float(rospy.get_param('~start_phase', 0.0))
        self.yaw_mode = rospy.get_param('~yaw_mode', 'tangent').strip().lower()
        self.figure8_scale_y = float(rospy.get_param('~figure8_scale_y', 0.5))
        self.spiral_climb_rate = float(rospy.get_param('~spiral_climb_rate', 0.2))
        self.startup_duration = max(0.0, float(rospy.get_param('~startup_duration', 2.5)))
        # A state-dependent LQR needs jerk to construct the complete desired
        # angular velocity, not only the desired yaw rate.
        self.publish_jerk = bool(rospy.get_param('~publish_jerk', True))
        self.start_from_current_position = bool(
            rospy.get_param('~start_from_current_position', True)
        )

        self.angular_speed = self.linear_speed / max(abs(self.radius), 1.0e-3)
        self.start_time = rospy.Time.now()
        self.trajectory_id = 0
        self.has_odom = not self.start_from_current_position
        self.origin_x = self.center_x
        self.origin_y = self.center_y
        self.origin_z = self.center_z
        self.initial_yaw = 0.0

        if self.start_from_current_position:
            self.odom_sub = rospy.Subscriber(
                '/mavros/local_position/odom', Odometry, self._odom_cb, queue_size=1
            )

        self.output_pub = rospy.Publisher(self.output_topic, PlannerOutput, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._on_timer)

        rospy.loginfo(
            'analytic_reference_publisher: type=%s topic=%s rate=%.1fHz horizon=%.2fs dt=%.3fs',
            self.trajectory_type,
            self.output_topic,
            self.publish_rate,
            self.horizon_duration,
            self.sample_dt,
        )

    def _odom_cb(self, msg):
        if self.has_odom:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.origin_x = x - self.radius * math.cos(self.start_phase)
        if self.trajectory_type == 'figure8':
            self.origin_y = y - self.radius * self.figure8_scale_y * math.sin(2.0 * self.start_phase)
        else:
            self.origin_y = y - self.radius * math.sin(self.start_phase)
        self.origin_z = z
        orientation = msg.pose.pose.orientation
        self.initial_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        self.start_time = rospy.Time.now()
        self.has_odom = True

    def spin(self):
        rospy.spin()

    def _on_timer(self, _event):
        if not self.has_odom:
            return

        now = rospy.Time.now()
        msg = PlannerOutput()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.trajectory_id = self.trajectory_id
        msg.is_horizon = True
        msg.trajectory_start_time = self.start_time
        msg.trajectory_status = PlannerOutput.TRAJECTORY_STATUS_READY

        sample_count = max(2, int(math.floor(self.horizon_duration / self.sample_dt)) + 1)
        elapsed = max(0.0, (now - self.start_time).to_sec())
        for idx in range(sample_count):
            dt = min(idx * self.sample_dt, self.horizon_duration)
            state = self._evaluate(elapsed + dt)
            msg.points.append(self._build_point(elapsed + dt, state))

        self.output_pub.publish(msg)

    def _build_point(self, time_from_start, state):
        point = PlannerOutputPoint()
        point.time_from_start = rospy.Duration.from_sec(time_from_start)
        point.valid_mask = (
            PlannerOutputPoint.VALID_POSITION
            | PlannerOutputPoint.VALID_VELOCITY
            | PlannerOutputPoint.VALID_ACCELERATION
            | PlannerOutputPoint.VALID_YAW
            | PlannerOutputPoint.VALID_YAW_RATE
        )

        point.position.x, point.position.y, point.position.z = state['position']
        point.velocity.x, point.velocity.y, point.velocity.z = state['velocity']
        point.acceleration.x, point.acceleration.y, point.acceleration.z = state['acceleration']
        point.yaw = state['yaw']
        point.yaw_rate = state['yaw_rate']

        if self.publish_jerk:
            point.valid_mask |= PlannerOutputPoint.VALID_JERK
            point.jerk.x, point.jerk.y, point.jerk.z = state['jerk']

        return point

    def _evaluate(self, t):
        if self.trajectory_type == 'circle':
            return self._evaluate_circle(t)
        if self.trajectory_type == 'figure8':
            return self._evaluate_figure8(t)
        if self.trajectory_type == 'spiral':
            return self._evaluate_spiral(t)
        rospy.logwarn_throttle(5.0, 'Unknown trajectory_type=%s, fallback to circle', self.trajectory_type)
        return self._evaluate_circle(t)

    def _evaluate_circle(self, t):
        progress, progress_rate, progress_acceleration, progress_jerk = \
            self._startup_progress(t)
        theta = self.start_phase + self.angular_speed * progress
        c = math.cos(theta)
        s = math.sin(theta)
        w = self.angular_speed
        r = self.radius
        theta_rate = w * progress_rate
        theta_acceleration = w * progress_acceleration
        theta_jerk = w * progress_jerk

        position = (
            self.origin_x + r * c,
            self.origin_y + r * s,
            self.origin_z,
        )
        velocity = (
            -r * theta_rate * s,
            r * theta_rate * c,
            0.0,
        )
        acceleration = (
            -r * (c * theta_rate ** 2 + s * theta_acceleration),
            -r * (s * theta_rate ** 2 - c * theta_acceleration),
            0.0,
        )
        jerk = (
            r * (s * theta_rate ** 3 - 3.0 * c * theta_rate * theta_acceleration
                 - s * theta_jerk),
            r * (-c * theta_rate ** 3 - 3.0 * s * theta_rate * theta_acceleration
                 + c * theta_jerk),
            0.0,
        )

        yaw, yaw_rate = self._circle_yaw(t, theta, theta_rate)
        return {
            'position': position,
            'velocity': velocity,
            'acceleration': acceleration,
            'jerk': jerk,
            'yaw': yaw,
            'yaw_rate': yaw_rate,
        }

    def _startup_progress(self, elapsed):
        """Return phase progress and its first three time derivatives.

        The initial ramp has speed 0 -> 1 without exceeding cruise speed.
        It intentionally delays the path phase by half of startup_duration,
        which preserves position continuity when switching to cruise speed.
        """
        if self.startup_duration <= 0.0 or elapsed >= self.startup_duration:
            delayed_time = elapsed - 0.5 * self.startup_duration
            return delayed_time, 1.0, 0.0, 0.0

        u = max(0.0, elapsed / self.startup_duration)
        # Integral of v(u) = 3u^2 - 2u^3.  v is monotonic on [0, 1].
        progress = self.startup_duration * (u ** 3 - 0.5 * u ** 4)
        progress_rate = 3.0 * u ** 2 - 2.0 * u ** 3
        progress_acceleration = (6.0 * u - 6.0 * u ** 2) / self.startup_duration
        progress_jerk = (6.0 - 12.0 * u) / (self.startup_duration ** 2)
        return progress, progress_rate, progress_acceleration, progress_jerk

    @staticmethod
    def _shortest_angular_distance(start, end):
        return math.atan2(math.sin(end - start), math.cos(end - start))

    def _circle_yaw(self, elapsed, theta, theta_rate):
        if self.yaw_mode == 'fixed':
            return 0.0, 0.0

        tangent_yaw = math.atan2(math.cos(theta), -math.sin(theta))
        if self.startup_duration <= 0.0 or elapsed >= self.startup_duration:
            return tangent_yaw, theta_rate

        final_theta = self.start_phase + self.angular_speed * 0.5 * self.startup_duration
        final_yaw = math.atan2(math.cos(final_theta), -math.sin(final_theta))
        return self._startup_yaw(elapsed, final_yaw, self.angular_speed)

    def _startup_yaw(self, elapsed, final_yaw, final_yaw_rate):
        """Smoothly connect the measured initial yaw to a tangent yaw.

        The quintic matches yaw, yaw rate, and yaw acceleration at both ends.
        It is shared by every analytic path that uses the startup speed ramp.
        """
        u = max(0.0, min(1.0, elapsed / self.startup_duration))
        delta_yaw = self._shortest_angular_distance(self.initial_yaw, final_yaw)
        final_yaw_rate_scaled = final_yaw_rate * self.startup_duration
        # Quintic: yaw(0)=initial, yaw'(0)=0, yaw''(0)=0,
        # yaw(1)=tangent, yaw'(1)=final_yaw_rate, yaw''(1)=0.
        a3 = 10.0 * delta_yaw - 4.0 * final_yaw_rate_scaled
        a4 = 7.0 * final_yaw_rate_scaled - 15.0 * delta_yaw
        a5 = 6.0 * delta_yaw - 3.0 * final_yaw_rate_scaled
        yaw = self.initial_yaw + a3 * u ** 3 + a4 * u ** 4 + a5 * u ** 5
        yaw_rate = (3.0 * a3 * u ** 2 + 4.0 * a4 * u ** 3 + 5.0 * a5 * u ** 4) \
            / self.startup_duration
        return yaw, yaw_rate

    def _evaluate_figure8(self, t):
        progress, progress_rate, progress_acceleration, progress_jerk = \
            self._startup_progress(t)
        theta = self.start_phase + self.angular_speed * progress
        c = math.cos(theta)
        s = math.sin(theta)
        c2 = math.cos(2.0 * theta)
        s2 = math.sin(2.0 * theta)
        w = self.angular_speed
        rx = self.radius
        ry = self.radius * self.figure8_scale_y
        theta_rate = w * progress_rate
        theta_acceleration = w * progress_acceleration
        theta_jerk = w * progress_jerk

        position = (
            self.origin_x + rx * c,
            self.origin_y + ry * s2,
            self.origin_z,
        )
        velocity = (
            -rx * theta_rate * s,
            2.0 * ry * theta_rate * c2,
            0.0,
        )
        acceleration = (
            -rx * (c * theta_rate ** 2 + s * theta_acceleration),
            -4.0 * ry * s2 * theta_rate ** 2
            + 2.0 * ry * c2 * theta_acceleration,
            0.0,
        )
        jerk = (
            rx * (s * theta_rate ** 3 - 3.0 * c * theta_rate * theta_acceleration
                  - s * theta_jerk),
            -8.0 * ry * c2 * theta_rate ** 3
            - 12.0 * ry * s2 * theta_rate * theta_acceleration
            + 2.0 * ry * c2 * theta_jerk,
            0.0,
        )

        yaw, yaw_rate = self._figure8_yaw(t, theta, theta_rate)
        return {
            'position': position,
            'velocity': velocity,
            'acceleration': acceleration,
            'jerk': jerk,
            'yaw': yaw,
            'yaw_rate': yaw_rate,
        }

    def _figure8_yaw(self, elapsed, theta, theta_rate):
        if self.yaw_mode == 'fixed':
            return 0.0, 0.0

        tangent_yaw, tangent_yaw_rate = self._figure8_tangent_yaw(theta, theta_rate)
        if self.startup_duration <= 0.0 or elapsed >= self.startup_duration:
            return tangent_yaw, tangent_yaw_rate

        final_theta = self.start_phase + self.angular_speed * 0.5 * self.startup_duration
        final_yaw, final_yaw_rate = self._figure8_tangent_yaw(
            final_theta, self.angular_speed
        )
        return self._startup_yaw(elapsed, final_yaw, final_yaw_rate)

    def _figure8_tangent_yaw(self, theta, theta_rate):
        """Return the figure-eight tangent yaw and its analytic time derivative."""
        s = math.sin(theta)
        c2 = math.cos(2.0 * theta)
        s2 = math.sin(2.0 * theta)
        rx = self.radius
        ry = self.radius * self.figure8_scale_y
        dx_dtheta = -rx * s
        dy_dtheta = 2.0 * ry * c2
        tangent_norm_sq = dx_dtheta ** 2 + dy_dtheta ** 2
        if tangent_norm_sq < 1.0e-12:
            return self.initial_yaw, 0.0

        yaw = math.atan2(dy_dtheta, dx_dtheta)
        # d/dtheta atan2(y', x') = (x' y'' - y' x'') / |p'|^2.
        ddx_dtheta = -rx * math.cos(theta)
        ddy_dtheta = -4.0 * ry * s2
        yaw_rate = (
            (dx_dtheta * ddy_dtheta - dy_dtheta * ddx_dtheta)
            / tangent_norm_sq * theta_rate
        )
        return yaw, yaw_rate

    def _evaluate_spiral(self, t):
        state = self._evaluate_circle(t)
        z = self.origin_z + self.spiral_climb_rate * t
        vx, vy, _ = state['velocity']
        ax, ay, _ = state['acceleration']
        jx, jy, _ = state['jerk']
        state['position'] = (state['position'][0], state['position'][1], z)
        state['velocity'] = (vx, vy, self.spiral_climb_rate)
        state['acceleration'] = (ax, ay, 0.0)
        state['jerk'] = (jx, jy, 0.0)
        return state

    def _yaw_from_velocity(self, velocity):
        if self.yaw_mode == 'fixed':
            return 0.0, 0.0

        vx, vy, _ = velocity
        speed_xy = math.hypot(vx, vy)
        if speed_xy < 1.0e-6:
            return 0.0, 0.0

        yaw = math.atan2(vy, vx)
        if self.trajectory_type == 'circle':
            return yaw, self.angular_speed
        return yaw, 0.0


if __name__ == '__main__':
    try:
        AnalyticReferencePublisher().spin()
    except rospy.ROSInterruptException:
        pass
