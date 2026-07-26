#!/usr/bin/env python3

"""OpenDrone planner adapter.

Normalizes planner-native outputs into opendrone/PlannerOutput.
"""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from quadrotor_msgs.msg import Bspline, PolynomialTrajectory, PositionCommand

from opendrone.msg import PlannerOutput, PlannerOutputPoint


class UniformBsplineEval:
    """Minimal De Boor evaluation for uniform B-splines and their derivatives."""

    def __init__(self, control_pts, order, knots):
        self.pts = control_pts
        self.order = order
        self.knots = knots

    def _de_boor(self, t):
        k = self.order
        knots = self.knots
        pts = self.pts
        n = len(pts) - 1

        t = np.clip(t, knots[k], knots[n + 1])

        idx = k
        for i in range(k, n + 1):
            if knots[i] <= t < knots[i + 1]:
                idx = i
                break
        else:
            idx = n

        d = [pts[j].copy() for j in range(idx - k, idx + 1)]
        for r in range(1, k + 1):
            for j in range(k, r - 1, -1):
                left = knots[idx - k + j]
                right = knots[idx + 1 + j - r]
                alpha = 0.0 if abs(right - left) < 1e-10 else (t - left) / (right - left)
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
        return d[k]

    def derivative(self):
        k = self.order
        pts = self.pts
        knots = self.knots
        n = len(pts) - 1
        new_pts = []
        for i in range(n):
            denom = knots[i + k + 1] - knots[i + 1]
            if abs(denom) < 1e-10:
                new_pts.append(np.zeros(pts.shape[1]))
            else:
                new_pts.append(k * (pts[i + 1] - pts[i]) / denom)
        return UniformBsplineEval(np.array(new_pts), k - 1, knots[1:-1])

    def get_time_range(self):
        k = self.order
        n = len(self.pts) - 1
        return self.knots[k], self.knots[n + 1]

    def evaluate(self, t):
        return self._de_boor(t)


class PolynomialEvaluator:
    """Evaluate piecewise polynomials whose coefficients are stored by descending power."""

    def __init__(self, coeffs, durations):
        self.coeffs = coeffs
        self.durations = durations
        self.total_duration = sum(self.durations)

    def evaluate(self, t, derivative_order=0):
        piece_idx, local_t = self._locate_piece(t)
        coeff = self.coeffs[piece_idx]
        return self._evaluate_poly(coeff, local_t, derivative_order)

    def _locate_piece(self, t):
        if not self.durations:
            return 0, 0.0
        if t <= 0.0:
            return 0, 0.0
        remaining = t
        for idx, duration in enumerate(self.durations):
            if remaining <= duration or idx == len(self.durations) - 1:
                return idx, min(max(remaining, 0.0), duration)
            remaining -= duration
        return len(self.durations) - 1, self.durations[-1]

    @staticmethod
    def _evaluate_poly(coeff, t, derivative_order):
        order = len(coeff) - 1
        value = 0.0
        for idx, c in enumerate(coeff):
            power = order - idx
            if power < derivative_order:
                continue
            scale = 1.0
            for k in range(derivative_order):
                scale *= (power - k)
            value += c * scale * (t ** (power - derivative_order))
        return value


class PlannerAdapterNode:
    def __init__(self):
        rospy.init_node('planner_adapter')

        self.adapter_type = rospy.get_param('~adapter_type', 'position_command').strip()
        self.input_topic = rospy.get_param('~input_topic', '')
        self.output_topic = rospy.get_param('~planner_output_topic', '/planner/output')
        self.publish_horizon_on_timer = bool(rospy.get_param('~publish_horizon_on_timer', False))
        self.sample_dt = float(rospy.get_param('~sample_dt', 0.02))
        self.horizon_points = int(rospy.get_param('~horizon_points', 60))
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.prefer_native_yaw = bool(rospy.get_param('~prefer_native_yaw', True))
        self.waypoint_yaw_from_motion = bool(rospy.get_param('~waypoint_yaw_from_motion', True))

        self.output_pub = rospy.Publisher(self.output_topic, PlannerOutput, queue_size=10)

        self._last_yaw = 0.0
        self._last_position = None
        self._cached_builder = None

        if self.sample_dt <= 0.0:
            self.sample_dt = 0.02
        if self.horizon_points < 1:
            self.horizon_points = 1

        self._setup_backend()

        if self.publish_horizon_on_timer:
            rospy.Timer(rospy.Duration(self.sample_dt), self._on_timer)

        rospy.loginfo(
            "planner_adapter: type=%s input=%s output=%s timer=%s",
            self.adapter_type,
            self.input_topic,
            self.output_topic,
            self.publish_horizon_on_timer,
        )
        rospy.spin()

    def _setup_backend(self):
        if self.adapter_type == 'position_command':
            topic = self.input_topic or '/planning/pos_cmd'
            rospy.Subscriber(topic, PositionCommand, self._position_command_cb, tcp_nodelay=True)
            self.input_topic = topic
        elif self.adapter_type == 'bspline':
            topic = self.input_topic or '/planning/bspline'
            rospy.Subscriber(topic, Bspline, self._bspline_cb, tcp_nodelay=True)
            self.input_topic = topic
        elif self.adapter_type == 'polynomial':
            topic = self.input_topic or '/planning_cmd/poly_traj'
            rospy.Subscriber(topic, PolynomialTrajectory, self._polynomial_cb, tcp_nodelay=True)
            self.input_topic = topic
        elif self.adapter_type == 'waypoint':
            topic = self.input_topic or '/way_point'
            rospy.Subscriber(topic, PointStamped, self._waypoint_cb, tcp_nodelay=True)
            self.input_topic = topic
        else:
            raise ValueError("Unsupported adapter_type: {}".format(self.adapter_type))

    def _on_timer(self, _event):
        if self._cached_builder is None:
            return
        planner_output = self._cached_builder(rospy.Time.now())
        if planner_output is None:
            return
        self._publish_outputs(planner_output)

    def _publish_outputs(self, planner_output):
        self.output_pub.publish(planner_output)

    @staticmethod
    def _copy_output_metadata(src, stamp):
        out = PlannerOutput()
        out.header.stamp = stamp
        out.header.frame_id = src.header.frame_id
        out.trajectory_id = src.trajectory_id
        out.is_horizon = src.is_horizon
        out.trajectory_start_time = src.trajectory_start_time
        return out

    def _new_output(self, stamp):
        msg = PlannerOutput()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        return msg

    def _make_point(
        self,
        time_from_start=0.0,
        position=None,
        velocity=None,
        acceleration=None,
        jerk=None,
        snap=None,
        yaw=None,
        yaw_rate=None,
        angular_velocity=None,
    ):
        point = PlannerOutputPoint()
        point.time_from_start = rospy.Duration.from_sec(max(float(time_from_start), 0.0))
        mask = 0

        if position is not None:
            point.position.x = float(position[0])
            point.position.y = float(position[1])
            point.position.z = float(position[2])
            mask |= PlannerOutputPoint.VALID_POSITION
        if velocity is not None:
            point.velocity.x = float(velocity[0])
            point.velocity.y = float(velocity[1])
            point.velocity.z = float(velocity[2])
            mask |= PlannerOutputPoint.VALID_VELOCITY
        if acceleration is not None:
            point.acceleration.x = float(acceleration[0])
            point.acceleration.y = float(acceleration[1])
            point.acceleration.z = float(acceleration[2])
            mask |= PlannerOutputPoint.VALID_ACCELERATION
        if jerk is not None:
            point.jerk.x = float(jerk[0])
            point.jerk.y = float(jerk[1])
            point.jerk.z = float(jerk[2])
            mask |= PlannerOutputPoint.VALID_JERK
        if snap is not None:
            point.snap.x = float(snap[0])
            point.snap.y = float(snap[1])
            point.snap.z = float(snap[2])
            mask |= PlannerOutputPoint.VALID_SNAP
        if yaw is not None:
            point.yaw = float(yaw)
            mask |= PlannerOutputPoint.VALID_YAW
        if yaw_rate is not None:
            point.yaw_rate = float(yaw_rate)
            mask |= PlannerOutputPoint.VALID_YAW_RATE
        if angular_velocity is not None:
            point.angular_velocity.x = float(angular_velocity[0])
            point.angular_velocity.y = float(angular_velocity[1])
            point.angular_velocity.z = float(angular_velocity[2])
            mask |= PlannerOutputPoint.VALID_ANGULAR_VELOCITY
        point.valid_mask = mask
        return point

    def _position_command_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        planner_output = self._new_output(stamp)
        planner_output.header.frame_id = msg.header.frame_id or self.frame_id
        planner_output.trajectory_id = int(msg.trajectory_id)
        planner_output.is_horizon = False
        planner_output.trajectory_start_time = stamp
        planner_output.points.append(
            self._make_point(
                time_from_start=0.0,
                position=(msg.position.x, msg.position.y, msg.position.z),
                velocity=(msg.velocity.x, msg.velocity.y, msg.velocity.z),
                acceleration=(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z),
                jerk=(msg.jerk.x, msg.jerk.y, msg.jerk.z),
                yaw=msg.yaw,
                yaw_rate=msg.yaw_dot,
                angular_velocity=(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            )
        )
        self._publish_outputs(planner_output)

    def _bspline_cb(self, msg):
        pts = np.array([[p.x, p.y, p.z] for p in msg.pos_pts], dtype=float)
        knots = np.array(msg.knots, dtype=float)
        if pts.size == 0 or knots.size == 0:
            rospy.logwarn_throttle(2.0, "planner_adapter: received empty bspline")
            return

        pos_spline = UniformBsplineEval(pts, msg.order, knots)
        vel_spline = pos_spline.derivative()
        acc_spline = vel_spline.derivative() if msg.order >= 2 else None
        jerk_spline = acc_spline.derivative() if acc_spline is not None and msg.order >= 3 else None
        yaw_spline = None
        if self.prefer_native_yaw and msg.yaw_pts and msg.yaw_dt > 0.0:
            yaw_coeff = np.array(msg.yaw_pts, dtype=float).reshape(-1, 1)
            yaw_knots = np.arange(len(msg.yaw_pts) + msg.order + 1, dtype=float) * float(msg.yaw_dt)
            if len(yaw_knots) >= msg.order + 2:
                yaw_spline = UniformBsplineEval(yaw_coeff, msg.order, yaw_knots)

        t0, t1 = pos_spline.get_time_range()

        def builder(stamp):
            rel_t = (stamp - msg.start_time).to_sec() + t0
            if rel_t > t1:
                return None
            planner_output = self._new_output(stamp)
            planner_output.trajectory_id = int(msg.traj_id)
            planner_output.is_horizon = (
                self.publish_horizon_on_timer or self.horizon_points > 1
            )
            planner_output.trajectory_start_time = msg.start_time

            sample_count = self.horizon_points if self.publish_horizon_on_timer else 1
            for idx in range(sample_count):
                sample_t = np.clip(rel_t + idx * self.sample_dt, t0, t1)
                pos = pos_spline.evaluate(sample_t)
                vel = vel_spline.evaluate(sample_t)
                acc = acc_spline.evaluate(sample_t) if acc_spline is not None else np.zeros(3)
                jerk = jerk_spline.evaluate(sample_t) if jerk_spline is not None else None
                yaw = None
                yaw_rate = None
                if yaw_spline is not None:
                    yaw = float(yaw_spline.evaluate(sample_t)[0])
                elif np.linalg.norm(vel[:2]) > 0.1:
                    yaw = math.atan2(vel[1], vel[0])
                else:
                    yaw = self._last_yaw
                if idx == 0 and yaw is not None:
                    self._last_yaw = yaw
                planner_output.points.append(
                    self._make_point(
                        time_from_start=(stamp - msg.start_time).to_sec() + idx * self.sample_dt,
                        position=pos,
                        velocity=vel,
                        acceleration=acc,
                        jerk=jerk,
                        yaw=yaw,
                        yaw_rate=yaw_rate,
                    )
                )
            return planner_output

        self._cached_builder = builder
        if not self.publish_horizon_on_timer:
            planner_output = builder(rospy.Time.now())
            if planner_output is not None:
                self._publish_outputs(planner_output)

    def _polynomial_cb(self, msg):
        stamp = rospy.Time.now()
        planner_output = self._new_output(stamp)
        planner_output.header.frame_id = msg.header.frame_id or self.frame_id
        planner_output.trajectory_id = int(msg.trajectory_id)
        planner_output.is_horizon = False
        planner_output.trajectory_start_time = msg.start_WT_pos

        has_pos = bool(msg.type & PolynomialTrajectory.POSITION_TRAJ) and bool(msg.time_pos)
        has_yaw = bool(msg.type & PolynomialTrajectory.YAW_TRAJ) and bool(msg.time_yaw)

        if not has_pos:
            self._cached_builder = None
            self._publish_outputs(planner_output)
            return

        pos_order = int(msg.order_pos)
        if len(msg.coef_pos_x) % (pos_order + 1) != 0 or len(msg.coef_pos_y) % (pos_order + 1) != 0 or len(msg.coef_pos_z) % (pos_order + 1) != 0:
            rospy.logwarn("planner_adapter: invalid polynomial coefficient layout on %s", self.input_topic)
            return
        coeffs_x = np.array(msg.coef_pos_x, dtype=float).reshape(-1, pos_order + 1)
        coeffs_y = np.array(msg.coef_pos_y, dtype=float).reshape(-1, pos_order + 1)
        coeffs_z = np.array(msg.coef_pos_z, dtype=float).reshape(-1, pos_order + 1)
        pos_coeffs = [np.vstack((coeffs_x[i], coeffs_y[i], coeffs_z[i])).T for i in range(coeffs_x.shape[0])]
        pos_durations = [float(t) for t in msg.time_pos]

        pos_eval_x = PolynomialEvaluator([coeff[:, 0] for coeff in pos_coeffs], pos_durations)
        pos_eval_y = PolynomialEvaluator([coeff[:, 1] for coeff in pos_coeffs], pos_durations)
        pos_eval_z = PolynomialEvaluator([coeff[:, 2] for coeff in pos_coeffs], pos_durations)

        yaw_eval = None
        if has_yaw:
            yaw_order = int(msg.order_yaw)
            if len(msg.coef_yaw) % (yaw_order + 1) != 0:
                rospy.logwarn("planner_adapter: invalid polynomial yaw coefficient layout on %s", self.input_topic)
                return
            yaw_coeffs = np.array(msg.coef_yaw, dtype=float).reshape(-1, yaw_order + 1)
            yaw_eval = PolynomialEvaluator([yaw_coeffs[i] for i in range(yaw_coeffs.shape[0])], [float(t) for t in msg.time_yaw])

        def builder(now):
            rel_t = (now - msg.start_WT_pos).to_sec()
            if rel_t < 0.0:
                rel_t = 0.0
            if rel_t > pos_eval_x.total_duration:
                return None
            out = self._copy_output_metadata(planner_output, now)
            out.is_horizon = self.publish_horizon_on_timer or self.horizon_points > 1

            sample_count = self.horizon_points if self.publish_horizon_on_timer else 1
            for idx in range(sample_count):
                sample_t = min(rel_t + idx * self.sample_dt, pos_eval_x.total_duration)
                pos = np.array([
                    pos_eval_x.evaluate(sample_t, 0),
                    pos_eval_y.evaluate(sample_t, 0),
                    pos_eval_z.evaluate(sample_t, 0),
                ])
                vel = np.array([
                    pos_eval_x.evaluate(sample_t, 1),
                    pos_eval_y.evaluate(sample_t, 1),
                    pos_eval_z.evaluate(sample_t, 1),
                ])
                acc = np.array([
                    pos_eval_x.evaluate(sample_t, 2),
                    pos_eval_y.evaluate(sample_t, 2),
                    pos_eval_z.evaluate(sample_t, 2),
                ])
                jerk = np.array([
                    pos_eval_x.evaluate(sample_t, 3),
                    pos_eval_y.evaluate(sample_t, 3),
                    pos_eval_z.evaluate(sample_t, 3),
                ])
                yaw = None
                yaw_rate = None
                if yaw_eval is not None:
                    yaw_t = min(max((now - msg.start_WT_yaw).to_sec(), 0.0) + idx * self.sample_dt, yaw_eval.total_duration)
                    yaw = yaw_eval.evaluate(yaw_t, 0)
                    yaw_rate = yaw_eval.evaluate(yaw_t, 1)
                out.points.append(
                    self._make_point(
                        time_from_start=rel_t + idx * self.sample_dt,
                        position=pos,
                        velocity=vel,
                        acceleration=acc,
                        jerk=jerk,
                        yaw=yaw,
                        yaw_rate=yaw_rate,
                    )
                )
            return out

        self._cached_builder = builder
        if not self.publish_horizon_on_timer:
            current_output = builder(rospy.Time.now())
            if current_output is not None:
                self._publish_outputs(current_output)

    def _waypoint_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        planner_output = self._new_output(stamp)
        planner_output.header.frame_id = msg.header.frame_id or self.frame_id
        planner_output.is_horizon = False
        planner_output.trajectory_start_time = stamp

        pos = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        yaw = self._last_yaw
        if self.waypoint_yaw_from_motion and self._last_position is not None:
            delta = pos - self._last_position
            if np.linalg.norm(delta[:2]) > 0.01:
                yaw = math.atan2(delta[1], delta[0])
        self._last_position = pos
        self._last_yaw = yaw

        planner_output.points.append(self._make_point(time_from_start=0.0, position=pos, yaw=yaw))
        self._publish_outputs(planner_output)


if __name__ == '__main__':
    PlannerAdapterNode()
