#!/usr/bin/env python3

import json
import tempfile
import unittest

import numpy as np

from flight_eval.analyzer import BagAnalyzer
from flight_eval.metrics import AnalysisReport, MetricResult, MetricsCalculator
from flight_eval.tasks import create_task


class TaskMetricModelTest(unittest.TestCase):
    def test_hover_bias_and_jitter_are_distinct_measurements(self):
        calculator = MetricsCalculator()
        target = np.array([0.0, 0.0, 2.0])
        positions = np.repeat([[0.3, -0.4, 2.0]], 20, axis=0)

        bias = calculator.compute_position_bias(positions, target)
        jitter = calculator.compute_position_jitter_rms(positions, target)

        self.assertAlmostEqual(bias.value, 0.5)
        self.assertAlmostEqual(jitter.value, 0.0)

    def test_task_emits_fixed_metric_names_when_data_is_unavailable(self):
        task = create_task('hover')
        metrics = task.compute_metrics(MetricsCalculator(), {}, {})

        self.assertEqual(
            [metric.name for metric in metrics],
            [
                'position_bias', 'position_jitter_rms', 'z_steady_state_error',
                'attitude_fluctuation_rms', 'thrust_std',
                'oscillation_pos_x', 'oscillation_pos_z', 'oscillation_roll',
                'oscillation_thrust',
            ],
        )
        self.assertTrue(all(not metric.available for metric in metrics))

    def test_report_schema_has_factual_metric_metadata_only(self):
        report = AnalysisReport(
            controller='controller', task='hover', bag_file='flight.bag',
            hover_height=2.0, flight_duration=1.0, execution_duration=1.0,
            metric_profile='hover_regulation',
            metrics=[MetricResult(
                name='position_bias', description='稳态位置偏差', value=0.1,
                unit='m', group='tracking', source='odometry + task target',
                phase='hover',
            )],
        )
        analyzer = BagAnalyzer.__new__(BagAnalyzer)
        with tempfile.TemporaryDirectory() as output_dir:
            path = analyzer.save_report(report, output_dir)
            with open(path, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)

        self.assertEqual(payload['format_version'], 2)
        self.assertEqual(payload['metric_profile'], 'hover_regulation')
        self.assertNotIn('benchmark_type', payload)
        self.assertNotIn('benchmark_dimensions', payload)
        self.assertNotIn('rating', payload['metrics'][0])
        self.assertEqual(payload['metrics'][0]['source'], 'odometry + task target')

    @staticmethod
    def _planner_point(stamp, position, velocity=(1.0, 0.0, 0.0),
                       acceleration=(0.0, 0.0, 0.0), jerk=(0.0, 0.0, 0.0),
                       valid_mask=15):
        return {
            'time': float(stamp),
            'time_from_start': float(stamp),
            'trajectory_id': 7,
            'valid_mask': valid_mask,
            'position': np.asarray(position, dtype=float),
            'velocity': np.asarray(velocity, dtype=float),
            'acceleration': np.asarray(acceleration, dtype=float),
            'jerk': np.asarray(jerk, dtype=float),
            'snap': np.full(3, np.nan),
            'yaw': float('nan'),
            'yaw_rate': float('nan'),
            'angular_velocity': np.full(3, np.nan),
        }

    def test_tracking_profile_has_new_metrics_without_steady_delay_or_z_duplicate(self):
        task = create_task('analytic_circle')
        metrics = task.compute_metrics(MetricsCalculator(), {}, {})
        names = [metric.name for metric in metrics]

        self.assertIn('position_tracking_rmse_3d', names)
        self.assertIn('position_tracking_p95_3d', names)
        self.assertIn('velocity_tracking_rmse_3d', names)
        self.assertIn('spatial_shape_rmse_3d', names)
        self.assertIn('spatial_shape_p95_3d', names)
        self.assertIn('reference_coverage_ratio', names)
        self.assertIn('max_reference_gap', names)
        self.assertIn('tracking_error_spectrum_peak', names)
        self.assertNotIn('steady_state_tracking_rmse', names)
        self.assertNotIn('z_tracking_rmse', names)
        self.assertNotIn('yaw_tracking_rmse', names)

    def test_spatial_shape_fidelity_is_independent_of_time_pairing(self):
        calculator = MetricsCalculator()
        times = np.arange(80, dtype=float) * 0.1
        angles = np.arange(80, dtype=float) * 2.0 * np.pi / 80.0
        reference = np.column_stack((np.cos(angles), np.sin(angles), angles * 0.0))
        actual = np.roll(reference, 20, axis=0)

        temporal = calculator.compute_position_tracking_metrics(
            actual, times, reference, times,
        )
        spatial = calculator.compute_spatial_shape_metrics(
            actual, times, reference, times,
        )

        self.assertGreater(temporal[0].value, 1.0)
        self.assertLess(spatial[0].value, 1e-10)
        self.assertEqual(spatial[0].group, 'spatial_fidelity')

    def test_yaw_metric_is_emitted_only_for_explicit_valid_yaw_reference(self):
        task = create_task('analytic_circle')
        times = np.linspace(0.0, 1.9, 20)
        phase = {
            'positions': np.column_stack((times, times * 0.0, times * 0.0)),
            'position_times': times,
            'velocities': np.repeat([[1.0, 0.0, 0.0]], len(times), axis=0),
            'velocity_times': times,
            'attitudes': np.column_stack((times * 0.0, times * 0.0, times * 0.0)),
            'attitude_times': times,
            'reference_positions': np.column_stack((times, times * 0.0, times * 0.0)),
            'reference_velocities': np.repeat([[1.0, 0.0, 0.0]], len(times), axis=0),
            'ref_pos_times': times,
            'reference_yaws': np.full(len(times), np.nan),
        }
        without_yaw = task.compute_metrics(MetricsCalculator(), phase, {})
        self.assertNotIn('yaw_tracking_rmse', [metric.name for metric in without_yaw])

        phase['reference_yaws'] = np.zeros(len(times))
        with_yaw = task.compute_metrics(MetricsCalculator(), phase, {})
        yaw_metric = next(metric for metric in with_yaw if metric.name == 'yaw_tracking_rmse')
        self.assertTrue(yaw_metric.available)
        self.assertAlmostEqual(yaw_metric.value, 0.0)

    def test_reference_gap_is_not_filled_by_interpolation(self):
        calculator = MetricsCalculator()
        times = np.arange(20, dtype=float)
        ref_times = np.concatenate((times[:5], times[15:]))
        refs = np.column_stack((ref_times, ref_times * 0.0, ref_times * 0.0))
        actual = np.column_stack((times, times * 0.0, times * 0.0))

        tracking = calculator.compute_position_tracking_metrics(actual, times, refs, ref_times)
        availability = calculator.compute_reference_availability(times, ref_times)

        self.assertTrue(all(metric.available for metric in tracking))
        self.assertEqual(tracking[0].detail['sample_count'], 10)
        self.assertAlmostEqual(availability[0].value, 0.5)
        self.assertAlmostEqual(availability[1].value, 10.0)

    def test_generation_metrics_use_continuous_polyline_distance_and_derivatives(self):
        calculator = MetricsCalculator()
        points = [
            self._planner_point(stamp, (stamp, 0.0, 0.0))
            for stamp in np.linspace(0.0, 10.0, 11)
        ]
        trajectories = [{
            'trajectory_id': 7,
            'is_horizon': False,
            'message_times': np.array([0.0]),
            'points': points,
        }]
        metrics = calculator.compute_trajectory_generation_metrics(
            trajectories, np.array([[5.0, 1.0, 0.0]]),
        )
        by_name = {metric.name: metric for metric in metrics}

        self.assertAlmostEqual(by_name['generated_trajectory_duration'].value, 10.0)
        self.assertAlmostEqual(by_name['waypoint_distance_mean'].value, 1.0)
        self.assertAlmostEqual(by_name['waypoint_distance_p95'].value, 1.0)
        self.assertAlmostEqual(by_name['waypoint_distance_max'].value, 1.0)
        self.assertAlmostEqual(by_name['speed_max'].value, 1.0)
        self.assertAlmostEqual(by_name['acceleration_max'].value, 0.0)
        self.assertAlmostEqual(by_name['jerk_rms'].value, 0.0)
        self.assertAlmostEqual(by_name['position_continuity_jump_max'].value, 0.0)

    def test_planner_trajectory_deduplication_keeps_latest_full_point(self):
        first = self._planner_point(1.0, (1.0, 0.0, 0.0))
        updated = self._planner_point(1.0, (1.0, 0.1, 0.0), valid_mask=7)
        windows = [
            {'trajectory_id': 7, 'message_time': 0.0, 'is_horizon': True,
             'points': [self._planner_point(0.0, (0.0, 0.0, 0.0)), first]},
            {'trajectory_id': 7, 'message_time': 0.5, 'is_horizon': True,
             'points': [updated, self._planner_point(2.0, (2.0, 0.0, 0.0))]},
        ]

        trajectories = BagAnalyzer._deduplicate_planner_trajectories(windows)

        self.assertEqual(len(trajectories), 1)
        self.assertEqual(len(trajectories[0]['points']), 3)
        middle = trajectories[0]['points'][1]
        self.assertEqual(middle['trajectory_id'], 7)
        self.assertEqual(middle['valid_mask'], 7)
        self.assertAlmostEqual(middle['time_from_start'], 1.0)
        np.testing.assert_allclose(middle['position'], [1.0, 0.1, 0.0])

    def test_odometry_child_frame_velocity_is_rotated_to_world_frame(self):
        half_angle = np.pi / 4.0
        rotated = BagAnalyzer._rotate_vector_by_quaternion(
            np.cos(half_angle), 0.0, 0.0, np.sin(half_angle),
            np.array([1.0, 0.0, 0.0]),
        )
        np.testing.assert_allclose(rotated, [0.0, 1.0, 0.0], atol=1e-12)

    def test_discrete_geometry_does_not_fall_back_to_actual_or_reference(self):
        task = create_task('discrete_circle')
        angles = np.linspace(0.0, 2.0 * np.pi, 40)
        circle = np.column_stack((np.cos(angles), np.sin(angles), angles * 0.0))
        phase = {
            'positions': circle,
            'position_times': np.arange(len(circle), dtype=float),
            'reference_positions': circle,
            'ref_pos_times': np.arange(len(circle), dtype=float),
            'planner_trajectories': [],
        }
        metrics = task.compute_metrics(
            MetricsCalculator(), phase, {'waypoint_positions': circle[::5]},
        )
        geometry = [metric for metric in metrics if metric.name.startswith('circle_')]
        self.assertTrue(geometry)
        self.assertTrue(all(not metric.available for metric in geometry))
        self.assertTrue(all(metric.detail.get('geometry_source') == 'planner_output_only'
                            for metric in geometry))


if __name__ == '__main__':
    unittest.main()
