#!/usr/bin/env python3

"""OpenDrone planner adapter.

Normalizes planner-native outputs into opendrone/PlannerOutput.
"""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from quadrotor_msgs.msg import Bspline, PolyTraj, PolynomialTrajectory, PositionCommand

from opendrone.msg import PlannerOutput, PlannerOutputPoint


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def shortest_angular_distance(source, target):
    """Return the shortest signed rotation from source to target."""
    return wrap_angle(float(target) - float(source))


def aligned_spline_time(sample_time, source_start, target_start, target_end):
    """Map a sample between independent knot domains by elapsed time."""
    return float(np.clip(
        float(target_start) + (float(sample_time) - float(source_start)),
        float(target_start),
        float(target_end),
    ))


class TrajectoryIdentityAllocator:
    """Turn planner-local identities into adapter-lifetime unique IDs.

    Native planners may restart their counters at a waypoint or after an FSM
    reset. PlannerOutput consumers may not: several controllers use the ID to
    reset their horizon cursor. The native start time is therefore part of the
    input identity, while the public ID is a monotonic adapter-local sequence.
    """

    def __init__(self):
        self._next_id = 1
        self._active_key = None
        self._active_id = 0

    @property
    def active_id(self):
        return self._active_id

    def normalize(self, native_id=None, start_time_ns=None, force_new=False):
        key = (native_id, start_time_ns)
        if force_new or self._active_key != key:
            self._active_key = key
            self._active_id = self._next_id
            self._next_id += 1
        return self._active_id


class YawReferenceFilter:
    """Generate a continuous, rate/acceleration-limited motion yaw profile."""

    def __init__(self, max_rate, max_acceleration, max_update_dt=0.1):
        self.max_rate = max(float(max_rate), 1.0e-3)
        self.max_acceleration = max(float(max_acceleration), 1.0e-3)
        self.max_update_dt = max(float(max_update_dt), 1.0e-3)
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.stamp = None
        self.initialized = False

    def anchor(self, yaw, yaw_rate, stamp):
        self.yaw = float(yaw)
        self.yaw_rate = float(np.clip(yaw_rate, -self.max_rate, self.max_rate))
        self.stamp = float(stamp)
        self.initialized = True

    def _step(self, yaw, yaw_rate, target, dt):
        dt = max(float(dt), 1.0e-4)
        error = shortest_angular_distance(yaw, target)
        desired_rate = float(np.clip(error / dt, -self.max_rate, self.max_rate))
        rate_delta = self.max_acceleration * dt
        next_rate = float(np.clip(
            desired_rate,
            yaw_rate - rate_delta,
            yaw_rate + rate_delta,
        ))
        step = next_rate * dt
        if abs(step) > abs(error):
            step = error
            next_rate = step / dt
        return yaw + step, next_rate

    def generate(self, targets, stamp, sample_dt):
        if not targets:
            return [], []

        stamp = float(stamp)
        if not self.initialized:
            yaw = float(targets[0])
            yaw_rate = 0.0
            first_yaw = yaw
            first_rate = yaw_rate
        else:
            dt = float(np.clip(
                stamp - float(self.stamp), 1.0e-4, self.max_update_dt
            ))
            yaw, yaw_rate = self._step(self.yaw, self.yaw_rate, targets[0], dt)
            first_yaw = yaw
            first_rate = yaw_rate

        yaws = [first_yaw]
        yaw_rates = [first_rate]
        for target in targets[1:]:
            yaw, yaw_rate = self._step(yaw, yaw_rate, target, sample_dt)
            yaws.append(yaw)
            yaw_rates.append(yaw_rate)

        # Commit only the present sample. Future horizon points must not move
        # the state used to construct the next rolling horizon.
        self.anchor(first_yaw, first_rate, stamp)
        return yaws, yaw_rates


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


class EgoV2PolyTrajEvaluator(PolynomialEvaluator):
    """Evaluate EGO-Planner v2 ``quadrotor_msgs/PolyTraj`` coefficients.

    The message preserves ``poly_traj::Piece::coeffMat`` column order from
    EGO v2: ``[c_order, ..., c_1, c_0]``.  For its quintic trajectory this is
    ``[c5, c4, c3, c2, c1, c0]`` and the native evaluator computes
    ``c5*t**5 + ... + c1*t + c0``.  This is already the descending-power
    convention used by :class:`PolynomialEvaluator`; coefficients must not be
    reversed when crossing this interface.

    Keeping this as a named evaluator makes the wire-format contract explicit
    instead of relying on an incidental generic-polynomial assumption.
    """

    def __init__(self, coeffs, durations):
        super().__init__([np.asarray(coeff, dtype=float) for coeff in coeffs], durations)


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
        self.position_command_topic = rospy.get_param('~position_command_topic', '').strip()
        self.position_command_timeout = float(rospy.get_param('~position_command_timeout', 0.2))
        self.yaw_rate_limit = float(rospy.get_param('~yaw_rate_limit', 1.5))
        self.yaw_acceleration_limit = float(rospy.get_param('~yaw_acceleration_limit', 3.0))
        self.yaw_update_dt_limit = float(rospy.get_param('~yaw_update_dt_limit', 0.1))
        self.motion_yaw_speed_threshold = float(rospy.get_param('~motion_yaw_speed_threshold', 0.1))
        self.output_pub = rospy.Publisher(self.output_topic, PlannerOutput, queue_size=10)

        self._last_yaw = 0.0
        self._last_position = None
        self._cached_builder = None
        self._identity = TrajectoryIdentityAllocator()
        self._yaw_filter = YawReferenceFilter(
            self.yaw_rate_limit,
            self.yaw_acceleration_limit,
            self.yaw_update_dt_limit,
        )
        self._latest_position_command = None
        self._latest_position_command_stamp = None
        self._latest_command_yaw_rate = 0.0

        if self.sample_dt <= 0.0:
            self.sample_dt = 0.02
        if self.horizon_points < 1:
            self.horizon_points = 1

        self._setup_backend()

        if self.adapter_type != 'position_command' and self.position_command_topic:
            rospy.Subscriber(
                self.position_command_topic,
                PositionCommand,
                self._position_command_state_cb,
                tcp_nodelay=True,
            )

        if self.publish_horizon_on_timer:
            rospy.Timer(rospy.Duration(self.sample_dt), self._on_timer)

        rospy.loginfo(
            "planner_adapter: type=%s input=%s output=%s timer=%s command_state=%s",
            self.adapter_type,
            self.input_topic,
            self.output_topic,
            self.publish_horizon_on_timer,
            self.position_command_topic or 'disabled',
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
        elif self.adapter_type == 'poly_traj':
            topic = self.input_topic or '/planning/trajectory'
            rospy.Subscriber(topic, PolyTraj, self._poly_traj_cb, tcp_nodelay=True)
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
        out.trajectory_status = src.trajectory_status
        return out

    def _new_output(self, stamp):
        msg = PlannerOutput()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.trajectory_status = PlannerOutput.TRAJECTORY_STATUS_READY
        return msg

    @staticmethod
    def _time_ns(stamp):
        return int(stamp.to_nsec())

    def _normalize_trajectory_id(self, native_id, start_time=None, force_new=False):
        start_ns = self._time_ns(start_time) if start_time is not None else None
        return self._identity.normalize(
            native_id=int(native_id),
            start_time_ns=start_ns,
            force_new=force_new,
        )

    def _position_command_state_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self._latest_position_command is not None and self._latest_position_command_stamp is not None:
            dt = (stamp - self._latest_position_command_stamp).to_sec()
            if dt > 1.0e-4:
                measured_rate = shortest_angular_distance(
                    self._latest_position_command.yaw, msg.yaw
                ) / dt
                rate_delta = self.yaw_acceleration_limit * dt
                self._latest_command_yaw_rate = float(np.clip(
                    measured_rate,
                    self._latest_command_yaw_rate - rate_delta,
                    self._latest_command_yaw_rate + rate_delta,
                ))
                self._latest_command_yaw_rate = float(np.clip(
                    self._latest_command_yaw_rate,
                    -self.yaw_rate_limit,
                    self.yaw_rate_limit,
                ))
        self._latest_position_command = msg
        self._latest_position_command_stamp = stamp

    def _fresh_command_yaw(self, stamp):
        if self._latest_position_command is None or self._latest_position_command_stamp is None:
            return None
        age = abs((stamp - self._latest_position_command_stamp).to_sec())
        if age > self.position_command_timeout:
            return None
        return (
            float(self._latest_position_command.yaw),
            self._latest_command_yaw_rate,
        )

    def _motion_yaw_profile(self, velocities, stamp):
        targets = []
        previous = self._yaw_filter.yaw if self._yaw_filter.initialized else self._last_yaw
        for velocity in velocities:
            if np.linalg.norm(velocity[:2]) > self.motion_yaw_speed_threshold:
                previous = math.atan2(velocity[1], velocity[0])
            targets.append(previous)
        native_yaw = self._fresh_command_yaw(stamp)
        if native_yaw is not None and targets:
            targets[0] = native_yaw[0]
        yaws, yaw_rates = self._yaw_filter.generate(
            targets,
            stamp.to_sec(),
            self.sample_dt,
        )
        if yaws:
            self._last_yaw = yaws[0]
        return yaws, yaw_rates

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

    def _output_from_position_command(self, msg, stamp):
        planner_output = self._new_output(stamp)
        planner_output.header.frame_id = msg.header.frame_id or self.frame_id
        planner_output.trajectory_id = self._normalize_trajectory_id(msg.trajectory_id)
        planner_output.is_horizon = False
        planner_output.trajectory_start_time = stamp
        # This field remains trajectory-local. Mission lifecycle is published
        # independently on /planner/mission_state by the mission owner.
        planner_output.trajectory_status = int(msg.trajectory_flag)
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
        return planner_output

    def _position_command_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        planner_output = self._output_from_position_command(msg, stamp)
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
        yaw_t0 = None
        yaw_t1 = None
        if self.prefer_native_yaw and msg.yaw_pts and msg.yaw_dt > 0.0:
            yaw_coeff = np.array(msg.yaw_pts, dtype=float).reshape(-1, 1)
            yaw_knots = np.arange(len(msg.yaw_pts) + msg.order + 1, dtype=float) * float(msg.yaw_dt)
            if len(yaw_knots) >= msg.order + 2:
                yaw_spline = UniformBsplineEval(yaw_coeff, msg.order, yaw_knots)
                yaw_t0, yaw_t1 = yaw_spline.get_time_range()

        t0, t1 = pos_spline.get_time_range()
        output_id = self._normalize_trajectory_id(msg.traj_id, msg.start_time)

        def builder(stamp):
            rel_t = (stamp - msg.start_time).to_sec() + t0
            if rel_t > t1:
                return None
            planner_output = self._new_output(stamp)
            planner_output.trajectory_id = output_id
            planner_output.is_horizon = (
                self.publish_horizon_on_timer or self.horizon_points > 1
            )
            planner_output.trajectory_start_time = msg.start_time

            sample_count = self.horizon_points if self.publish_horizon_on_timer else 1
            sample_times = [
                float(np.clip(rel_t + idx * self.sample_dt, t0, t1))
                for idx in range(sample_count)
            ]
            positions = [pos_spline.evaluate(sample_t) for sample_t in sample_times]
            velocities = [vel_spline.evaluate(sample_t) for sample_t in sample_times]
            accelerations = [
                acc_spline.evaluate(sample_t) if acc_spline is not None else np.zeros(3)
                for sample_t in sample_times
            ]
            jerks = [
                jerk_spline.evaluate(sample_t) if jerk_spline is not None else None
                for sample_t in sample_times
            ]

            if yaw_spline is not None:
                raw_yaws = []
                for sample_t in sample_times:
                    # Position and yaw splines have independent knot vectors.
                    # Align them by elapsed trajectory time, as the native
                    # traj_server's evaluateDeBoorT(t_cur) does.
                    yaw_t = aligned_spline_time(sample_t, t0, yaw_t0, yaw_t1)
                    raw_yaws.append(float(yaw_spline.evaluate(yaw_t)[0]))
                yaws, yaw_rates = self._yaw_filter.generate(
                    raw_yaws, stamp.to_sec(), self.sample_dt
                )
                self._last_yaw = yaws[0]
            else:
                yaws, yaw_rates = self._motion_yaw_profile(velocities, stamp)

            for idx, sample_t in enumerate(sample_times):
                planner_output.points.append(
                    self._make_point(
                        time_from_start=sample_t - t0,
                        position=positions[idx],
                        velocity=velocities[idx],
                        acceleration=accelerations[idx],
                        jerk=jerks[idx],
                        yaw=yaws[idx],
                        yaw_rate=yaw_rates[idx],
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
        planner_output.trajectory_id = self._normalize_trajectory_id(
            msg.trajectory_id, msg.start_WT_pos
        )
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
            sampled = []
            raw_yaws = []
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
                if yaw_eval is not None:
                    yaw_t = min(max((now - msg.start_WT_yaw).to_sec(), 0.0) + idx * self.sample_dt, yaw_eval.total_duration)
                    raw_yaws.append(yaw_eval.evaluate(yaw_t, 0))
                sampled.append((sample_t, pos, vel, acc, jerk))

            if yaw_eval is not None:
                yaws, yaw_rates = self._yaw_filter.generate(
                    raw_yaws, now.to_sec(), self.sample_dt
                )
            else:
                yaws, yaw_rates = self._motion_yaw_profile(
                    [item[2] for item in sampled], now
                )
            for idx, (sample_t, pos, vel, acc, jerk) in enumerate(sampled):
                out.points.append(
                    self._make_point(
                        time_from_start=sample_t,
                        position=pos,
                        velocity=vel,
                        acceleration=acc,
                        jerk=jerk,
                        yaw=yaws[idx],
                        yaw_rate=yaw_rates[idx],
                    )
                )
            return out

        self._cached_builder = builder
        if not self.publish_horizon_on_timer:
            current_output = builder(rospy.Time.now())
            if current_output is not None:
                self._publish_outputs(current_output)

    def _poly_traj_cb(self, msg):
        """Adapt EGO-Planner v2's PolyTraj message into a timed horizon.

        EGO v2 publishes the native ``poly_traj::Piece::coeffMat`` order,
        descending by power.  Sampling is intentionally equivalent to its
        ``getPos/getVel/getAcc/getJer`` methods; yaw is handled separately
        below because PolyTraj itself has no yaw trajectory fields.
        """
        order = int(msg.order)
        coefficient_count = order + 1
        durations = [float(duration) for duration in msg.duration]
        segment_count = len(durations)
        expected_count = segment_count * coefficient_count
        # EGO v2's own traj_server rejects non-quintic PolyTraj messages.
        # Do the same so this adapter cannot silently diverge from its native
        # trajectory consumer.
        if (order != 5 or not durations or any(duration <= 0.0 for duration in durations) or
                len(msg.coef_x) != expected_count or
                len(msg.coef_y) != expected_count or
                len(msg.coef_z) != expected_count):
            rospy.logwarn_throttle(
                2.0, 'planner_adapter: invalid EGO v2 quintic PolyTraj layout on %s', self.input_topic
            )
            return

        coefficients_x = np.asarray(msg.coef_x, dtype=float).reshape(segment_count, coefficient_count)
        coefficients_y = np.asarray(msg.coef_y, dtype=float).reshape(segment_count, coefficient_count)
        coefficients_z = np.asarray(msg.coef_z, dtype=float).reshape(segment_count, coefficient_count)
        position_x = EgoV2PolyTrajEvaluator(coefficients_x, durations)
        position_y = EgoV2PolyTrajEvaluator(coefficients_y, durations)
        position_z = EgoV2PolyTrajEvaluator(coefficients_z, durations)
        output_id = self._normalize_trajectory_id(msg.traj_id, msg.start_time)

        def builder(now):
            elapsed = max(0.0, (now - msg.start_time).to_sec())
            if elapsed > position_x.total_duration:
                return None

            output = self._new_output(now)
            output.trajectory_id = output_id
            output.is_horizon = self.publish_horizon_on_timer or self.horizon_points > 1
            output.trajectory_start_time = msg.start_time
            sample_count = self.horizon_points if self.publish_horizon_on_timer else 1
            sampled = []
            for index in range(sample_count):
                sample_t = min(elapsed + index * self.sample_dt, position_x.total_duration)
                position = np.array([
                    position_x.evaluate(sample_t, 0),
                    position_y.evaluate(sample_t, 0),
                    position_z.evaluate(sample_t, 0),
                ])
                velocity = np.array([
                    position_x.evaluate(sample_t, 1),
                    position_y.evaluate(sample_t, 1),
                    position_z.evaluate(sample_t, 1),
                ])
                acceleration = np.array([
                    position_x.evaluate(sample_t, 2),
                    position_y.evaluate(sample_t, 2),
                    position_z.evaluate(sample_t, 2),
                ])
                jerk = np.array([
                    position_x.evaluate(sample_t, 3),
                    position_y.evaluate(sample_t, 3),
                    position_z.evaluate(sample_t, 3),
                ])
                sampled.append((sample_t, position, velocity, acceleration, jerk))

            yaws, yaw_rates = self._motion_yaw_profile(
                [item[2] for item in sampled], now
            )
            for index, (sample_t, position, velocity, acceleration, jerk) in enumerate(sampled):
                output.points.append(self._make_point(
                    time_from_start=sample_t,
                    position=position,
                    velocity=velocity,
                    acceleration=acceleration,
                    jerk=jerk,
                    yaw=yaws[index],
                    yaw_rate=yaw_rates[index],
                ))
            return output

        self._cached_builder = builder
        if not self.publish_horizon_on_timer:
            planner_output = builder(rospy.Time.now())
            if planner_output is not None:
                self._publish_outputs(planner_output)

    def _waypoint_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        planner_output = self._new_output(stamp)
        planner_output.header.frame_id = msg.header.frame_id or self.frame_id
        planner_output.trajectory_id = self._identity.normalize(force_new=True)
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
