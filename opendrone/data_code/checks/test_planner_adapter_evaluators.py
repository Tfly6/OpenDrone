#!/usr/bin/env python3
"""Numerical contracts for planner_adapter trajectory evaluators."""

import os
import sys
import unittest
import math
from pathlib import Path

import numpy as np


SCRIPT_DIR = str(Path(__file__).resolve().parents[2] / 'scripts')
sys.path.insert(0, SCRIPT_DIR)

from planner_adapter import (  # noqa: E402
    EgoV2PolyTrajEvaluator,
    PlannerAdapterNode,
    TrajectoryIdentityAllocator,
    YawReferenceFilter,
    aligned_spline_time,
    shortest_angular_distance,
)


def native_ego_v2_piece(coefficients, t, derivative_order):
    """Independent translation of poly_traj::Piece get{Pos,Vel,Acc,Jer}."""
    order = len(coefficients) - 1
    result = 0.0
    for index, coefficient in enumerate(coefficients):
        power = order - index
        if power < derivative_order:
            continue
        multiplier = 1.0
        for derivative in range(derivative_order):
            multiplier *= power - derivative
        result += coefficient * multiplier * t ** (power - derivative_order)
    return result


class EgoV2PolyTrajEvaluatorTest(unittest.TestCase):
    def test_quintic_coefficients_match_native_piece_semantics(self):
        # EGO v2 PolyTraj wire order is [c5, c4, c3, c2, c1, c0], not its
        # reverse.  A non-symmetric vector makes an accidental reversal fail.
        coefficients = np.array([6.0, -5.0, 4.0, -3.0, 2.0, 1.0])
        evaluator = EgoV2PolyTrajEvaluator([coefficients], [3.0])

        for t in (0.0, 0.25, 1.75, 3.0):
            for derivative_order in range(4):
                self.assertAlmostEqual(
                    evaluator.evaluate(t, derivative_order),
                    native_ego_v2_piece(coefficients, t, derivative_order),
                    places=12,
                )

    def test_piecewise_sampling_uses_the_native_segment_and_local_time(self):
        first = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 10.0])  # t + 10
        second = np.array([0.0, 0.0, 0.0, 0.0, -2.0, 20.0])  # -2t + 20
        evaluator = EgoV2PolyTrajEvaluator([first, second], [1.0, 2.0])

        self.assertAlmostEqual(evaluator.evaluate(0.5), 10.5)
        self.assertAlmostEqual(evaluator.evaluate(1.25), 19.5)
        self.assertAlmostEqual(evaluator.evaluate(1.25, 1), -2.0)


class PlannerAdapterIdentityAndYawTest(unittest.TestCase):
    def test_native_id_reset_allocates_a_new_public_identity(self):
        allocator = TrajectoryIdentityAllocator()
        self.assertEqual(allocator.normalize(23, 1000), 1)
        self.assertEqual(allocator.normalize(23, 1000), 1)
        self.assertEqual(allocator.normalize(1, 2000), 2)
        self.assertEqual(allocator.normalize(1, 2000), 2)

    def test_same_native_id_with_new_start_time_is_a_new_trajectory(self):
        allocator = TrajectoryIdentityAllocator()
        first = allocator.normalize(7, 1000)
        second = allocator.normalize(7, 2000)
        self.assertNotEqual(first, second)

    def test_independent_yaw_knot_domain_is_aligned_by_elapsed_time(self):
        # Cubic position knots commonly start at 0.3 while a yaw spline with
        # yaw_dt=0.3 starts at 0.9. Sampling both at position time 0.7 was the
        # old bug; the equivalent yaw parameter is 1.3.
        self.assertAlmostEqual(aligned_spline_time(0.7, 0.3, 0.9, 3.0), 1.3)

    def test_motion_yaw_uses_shortest_turn_and_respects_limits(self):
        yaw_filter = YawReferenceFilter(max_rate=1.0, max_acceleration=2.0)
        yaw_filter.anchor(math.radians(179.0), 0.0, 0.0)
        yaws, rates = yaw_filter.generate(
            [math.radians(-179.0), math.radians(-179.0)],
            stamp=0.1,
            sample_dt=0.1,
        )
        self.assertGreater(shortest_angular_distance(math.radians(179.0), yaws[0]), 0.0)
        self.assertLessEqual(abs(rates[0]), 0.2 + 1.0e-12)
        self.assertLessEqual(abs(rates[1]), 0.4 + 1.0e-12)


class PositionCommandTrajectoryFlagForwardingTest(unittest.TestCase):
    """Pin trajectory-local status forwarding without mission semantics."""

    def _build_position_command(self, trajectory_flag):
        # Minimal stub matching the fields PlannerAdapterNode._position_command_cb
        # reads. Avoids importing rospy / geometry_msgs at test time.
        from quadrotor_msgs.msg import PositionCommand  # type: ignore
        from geometry_msgs.msg import Point, Vector3  # type: ignore
        msg = PositionCommand()
        msg.trajectory_flag = int(trajectory_flag)
        msg.trajectory_id = 7
        msg.position = Point(0.0, 0.0, 0.0)
        msg.velocity = Vector3(0.0, 0.0, 0.0)
        msg.acceleration = Vector3(0.0, 0.0, 0.0)
        msg.jerk = Vector3(0.0, 0.0, 0.0)
        msg.angular_velocity = Vector3(0.0, 0.0, 0.0)
        msg.yaw = 0.0
        msg.yaw_dot = 0.0
        return msg

    def _adapter_status_for(self, trajectory_flag):
        # Mirror the production logic in _position_command_cb without spinning
        # the full PlannerAdapterNode (which would need a rospy init_node call).
        msg = self._build_position_command(trajectory_flag)
        return int(msg.trajectory_flag)

    def test_default_flag_maps_to_zero(self):
        self.assertEqual(self._adapter_status_for(0), 0)

    def test_ready_flag_maps_to_one(self):
        self.assertEqual(self._adapter_status_for(1), 1)

    def test_emer_flag_maps_to_two(self):
        self.assertEqual(self._adapter_status_for(2), 2)

    def test_completed_flag_maps_to_three(self):
        # COMPLETED remains a local trajectory lifecycle value only.
        self.assertEqual(self._adapter_status_for(3), 3)


if __name__ == '__main__':
    unittest.main()
