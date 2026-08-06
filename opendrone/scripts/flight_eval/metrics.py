#!/usr/bin/env python3
"""Flight performance metric calculations."""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class MetricResult:
    """单个事实指标；不携带阈值、评级或综合结论。"""
    name: str
    description: str
    value: float
    unit: str
    group: str = ''
    source: str = ''
    phase: str = ''
    available: bool = True
    detail: Dict = field(default_factory=dict)
    def __post_init__(self) -> None:
        if not self.group:
            self.group = 'diagnostic'
        if not self.source:
            self.source = 'derived from recorded data'
        if isinstance(self.value, float) and math.isnan(self.value):
            self.available = False


@dataclass
class AnalysisReport:
    """完整分析报告"""
    controller: str
    task: str
    bag_file: str
    hover_height: float
    flight_duration: float
    execution_duration: float
    metric_profile: str = ''
    analysis_phase: str = ''
    run_status: Dict[str, str] = field(default_factory=dict)
    task_outcome: Dict[str, object] = field(default_factory=dict)
    metrics: List[MetricResult] = field(default_factory=list)
    summary: str = ''
    emergency_occurred: bool = False
    emergency_detail: Dict = field(default_factory=dict)
    artifacts: Dict = field(default_factory=dict)


class MetricsCalculator:
    """指标计算器"""

    def __init__(self, hover_height: float = 2.0):
        self.hover_height = hover_height

    @staticmethod
    def _wrap_angle(angle: np.ndarray) -> np.ndarray:
        """将角度包裹到 [-pi, pi]。"""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    @staticmethod
    def _align_reference_samples(
        sample_values: np.ndarray,
        sample_times: np.ndarray,
        reference_values: np.ndarray,
        reference_times: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """只对齐真正存在的时序参考，不跨 reference gap 插值。"""
        if not len(sample_values) or not len(sample_times) or not len(reference_values) or not len(reference_times):
            width = sample_values.shape[1] if np.ndim(sample_values) == 2 else 1
            return np.empty((0, width)), np.empty((0, width)), np.empty(0)
        reference_times = np.asarray(reference_times, dtype=float)
        order = np.argsort(reference_times, kind='stable')
        reference_times = reference_times[order]
        reference_values = np.asarray(reference_values)[order]
        indices = np.searchsorted(reference_times, sample_times)
        right = np.clip(indices, 0, len(reference_times) - 1)
        left = np.clip(indices - 1, 0, len(reference_times) - 1)
        choose_left = np.abs(reference_times[left] - sample_times) <= np.abs(reference_times[right] - sample_times)
        nearest = np.where(choose_left, left, right)
        if len(sample_times) > 1:
            positive_dt = np.diff(np.asarray(sample_times, dtype=float))
            positive_dt = positive_dt[positive_dt > 0.0]
            tolerance = max(1e-6, float(np.median(positive_dt)) * 1e-3) if len(positive_dt) else 1e-6
        else:
            tolerance = 1e-6
        valid = np.abs(reference_times[nearest] - sample_times) <= tolerance
        aligned_samples = np.asarray(sample_values)[valid]
        aligned_references = reference_values[nearest[valid]]
        finite = np.all(np.isfinite(aligned_samples), axis=1) & np.all(np.isfinite(aligned_references), axis=1)
        return aligned_samples[finite], aligned_references[finite], np.asarray(sample_times)[valid][finite]

    @staticmethod
    def _tracking_detail(errors: np.ndarray) -> Dict:
        norms = np.linalg.norm(errors, axis=1)
        detail = {
            'mean_error_3d': float(np.mean(norms)),
            'max_error_3d': float(np.max(norms)),
            'p95_error_3d': float(np.percentile(norms, 95)),
            'sample_count': int(len(norms)),
        }
        for axis, index in zip(('x', 'y', 'z'), range(3)):
            component = errors[:, index]
            detail.update({
                f'rmse_{axis}': float(np.sqrt(np.mean(component ** 2))),
                f'mean_error_{axis}': float(np.mean(component)),
                f'max_abs_error_{axis}': float(np.max(np.abs(component))),
            })
        return detail

    def compute_position_tracking_metrics(
        self,
        positions: np.ndarray,
        position_times: np.ndarray,
        reference_positions: np.ndarray,
        ref_pos_times: np.ndarray
    ) -> List[MetricResult]:
        """计算显式时序位置参考的 3D RMSE 和 P95。"""
        names = (
            ('position_tracking_rmse_3d', '位置跟踪 RMSE (3D)', 'rmse'),
            ('position_tracking_p95_3d', '位置跟踪 P95 (3D)', 'p95'),
        )
        if len(positions) < 10 or len(reference_positions) < 2:
            return [MetricResult(name=name, description=description, value=float('nan'),
                                 unit='m', group='tracking',
                                 source='odometry + active PlannerOutput reference',
                                 detail={'error': '位置或显式时序参考数据点不足'})
                    for name, description, _ in names]
        actual, reference, _ = self._align_reference_samples(
            positions, position_times, reference_positions, ref_pos_times,
        )
        if len(actual) < 10:
            return [MetricResult(name=name, description=description, value=float('nan'),
                                 unit='m', group='tracking',
                                 source='odometry + active PlannerOutput reference',
                                 detail={'error': '位置与有效参考重叠不足'})
                    for name, description, _ in names]
        errors = actual - reference
        norms = np.linalg.norm(errors, axis=1)
        detail = self._tracking_detail(errors)
        values = (
            float(np.sqrt(np.mean(norms ** 2))),
            float(np.percentile(norms, 95)),
        )
        return [MetricResult(name=name, description=description, value=value,
                             unit='m', group='tracking',
                             source='odometry + active PlannerOutput reference', detail=dict(detail))
                for (name, description, _), value in zip(names, values)]

    def compute_velocity_tracking_error(
        self,
        velocities: np.ndarray,
        velocity_times: np.ndarray,
        reference_velocities: np.ndarray,
        ref_pos_times: np.ndarray,
    ) -> MetricResult:
        """计算实际速度相对显式速度参考的 3D RMSE。"""
        if len(velocities) < 10 or len(reference_velocities) < 2:
            return MetricResult(
                name='velocity_tracking_rmse_3d',
                description='速度跟踪 RMSE (3D)',
                value=float('nan'),
                unit='m/s',
                group='tracking',
                source='odometry + active PlannerOutput reference',
                detail={'error': '速度或显式速度参考数据点不足'}
            )
        actual, reference, _ = self._align_reference_samples(
            velocities, velocity_times, reference_velocities, ref_pos_times,
        )
        if len(actual) < 10:
            return MetricResult(
                name='velocity_tracking_rmse_3d',
                description='速度跟踪 RMSE (3D)',
                value=float('nan'),
                unit='m/s',
                group='tracking',
                source='odometry + active PlannerOutput reference',
                detail={'error': '速度与有效显式参考重叠不足'}
            )
        errors = actual - reference
        norms = np.linalg.norm(errors, axis=1)
        return MetricResult(
            name='velocity_tracking_rmse_3d',
            description='速度跟踪 RMSE (3D)',
            value=float(np.sqrt(np.mean(norms ** 2))),
            unit='m/s',
            group='tracking',
            source='odometry + active PlannerOutput reference',
            detail=self._tracking_detail(errors),
        )

    def compute_reference_availability(
        self,
        sample_times: np.ndarray,
        reference_times: np.ndarray,
    ) -> List[MetricResult]:
        """计算参考覆盖率和最长缺口，缺口不会被插值掩盖。"""
        if len(sample_times) < 2:
            reason = {'error': '状态时间样本不足'}
            return [
                MetricResult(name='reference_coverage_ratio', description='参考覆盖率',
                             value=float('nan'), unit='ratio', group='diagnostic', detail=reason),
                MetricResult(name='max_reference_gap', description='最大参考缺口',
                             value=float('nan'), unit='s', group='diagnostic', detail=reason),
            ]
        sample_times = np.asarray(sample_times, dtype=float)
        dummy_samples = np.zeros((len(sample_times), 1))
        dummy_refs = np.zeros((len(reference_times), 1))
        _, _, covered_times = self._align_reference_samples(
            dummy_samples, sample_times, dummy_refs, reference_times,
        )
        covered = np.isin(sample_times, covered_times)
        coverage = float(np.mean(covered))
        median_dt = float(np.median(np.diff(sample_times)))
        gap = 0.0
        start = None
        for index, is_covered in enumerate(covered):
            if not is_covered and start is None:
                start = index
            if is_covered and start is not None:
                gap = max(gap, float(sample_times[index] - sample_times[start]))
                start = None
        if start is not None:
            gap = max(gap, float(sample_times[-1] - sample_times[start] + median_dt))
        detail = {
            'covered_sample_count': int(np.count_nonzero(covered)),
            'state_sample_count': int(len(sample_times)),
        }
        return [
            MetricResult(name='reference_coverage_ratio', description='参考覆盖率',
                         value=coverage, unit='ratio', group='diagnostic',
                         source='odometry timestamps + active PlannerOutput reference', detail=dict(detail)),
            MetricResult(name='max_reference_gap', description='最大参考缺口',
                         value=gap, unit='s', group='diagnostic',
                         source='odometry timestamps + active PlannerOutput reference', detail=dict(detail)),
        ]

    def compute_spatial_shape_metrics(
        self,
        positions: np.ndarray,
        position_times: np.ndarray,
        reference_positions: np.ndarray,
        reference_times: np.ndarray,
    ) -> List[MetricResult]:
        definitions = (
            ('spatial_shape_rmse_3d', '空间形状双向 RMSE (3D)'),
            ('spatial_shape_p95_3d', '空间形状双向 P95 (3D)'),
        )
        actual, reference, times = self._align_reference_samples(
            positions, position_times, reference_positions, reference_times,
        )
        if len(actual) < 3:
            return [MetricResult(
                name=name, description=description, value=float('nan'), unit='m',
                group='spatial_fidelity', source='actual/reference continuous polylines',
                detail={'error': '有效轨迹采样点不足'},
            ) for name, description in definitions]

        positive_dt = np.diff(times)
        positive_dt = positive_dt[positive_dt > 0.0]
        gap_threshold = 3.0 * float(np.median(positive_dt)) if len(positive_dt) else float('inf')
        boundaries = np.where(np.diff(times) > gap_threshold)[0] + 1
        blocks = np.split(np.arange(len(times)), boundaries)
        actual_to_reference = []
        reference_to_actual = []
        for indices in blocks:
            if len(indices) < 2:
                continue
            actual_to_reference.extend(self._point_to_polyline_distances(
                actual[indices], reference[indices],
            ))
            reference_to_actual.extend(self._point_to_polyline_distances(
                reference[indices], actual[indices],
            ))

        forward = np.asarray(actual_to_reference, dtype=float)
        reverse = np.asarray(reference_to_actual, dtype=float)
        distances = np.concatenate((forward, reverse)) if len(forward) and len(reverse) else np.empty(0)
        if not len(distances):
            return [MetricResult(
                name=name, description=description, value=float('nan'), unit='m',
                group='spatial_fidelity', source='actual/reference continuous polylines',
                detail={'error': '连续轨迹段不足'},
            ) for name, description in definitions]

        detail = {
            'actual_to_reference_rmse': float(np.sqrt(np.mean(forward ** 2))),
            'actual_to_reference_p95': float(np.percentile(forward, 95)),
            'reference_to_actual_rmse': float(np.sqrt(np.mean(reverse ** 2))),
            'reference_to_actual_p95': float(np.percentile(reverse, 95)),
            'sample_count_each_direction': int(len(forward)),
            'continuous_block_count': int(sum(len(indices) >= 2 for indices in blocks)),
            'time_pairing': 'ignored within each continuous reference block',
        }
        values = (
            float(np.sqrt(np.mean(distances ** 2))),
            float(np.percentile(distances, 95)),
        )
        return [MetricResult(
            name=name, description=description, value=value, unit='m',
            group='spatial_fidelity', source='actual/reference continuous polylines',
            detail=dict(detail),
        ) for (name, description), value in zip(definitions, values)]

    @staticmethod
    def select_primary_trajectory(trajectories: List[Dict]) -> Optional[Dict]:
        """选择采样时间跨度最大的一条轨迹；不跨 trajectory_id 拼接。"""
        candidates = []
        for trajectory in trajectories:
            points = trajectory.get('points', [])
            if len(points) < 2:
                continue
            times = np.asarray([point['time'] for point in points], dtype=float)
            candidates.append((float(np.max(times) - np.min(times)), len(points), trajectory))
        return max(candidates, key=lambda item: (item[0], item[1]))[2] if candidates else None

    @staticmethod
    def _point_to_polyline_distances(waypoints: np.ndarray, path: np.ndarray) -> np.ndarray:
        """计算 waypoint 到连续折线的最短距离，而非离散采样点命中。"""
        if len(waypoints) == 0 or len(path) < 2:
            return np.empty(0)
        starts = path[:-1]
        segments = path[1:] - starts
        lengths_sq = np.sum(segments ** 2, axis=1)
        delta = waypoints[:, None, :] - starts[None, :, :]
        projections = np.zeros((len(waypoints), len(starts)), dtype=float)
        nonzero = lengths_sq > 1e-12
        projections[:, nonzero] = (
            np.sum(delta[:, nonzero, :] * segments[None, nonzero, :], axis=2)
            / lengths_sq[nonzero]
        )
        projections = np.clip(projections, 0.0, 1.0)
        closest = starts[None, :, :] + projections[:, :, None] * segments[None, :, :]
        return np.min(np.linalg.norm(waypoints[:, None, :] - closest, axis=2), axis=1)

    @staticmethod
    def _continuity_residual(
        points: List[Dict], value_key: str, derivative_key: str,
    ) -> np.ndarray:
        """以梯形积分残差度量相邻采样间的最大连续性跳变。"""
        residuals = []
        for left, right in zip(points[:-1], points[1:]):
            dt = float(right['time'] - left['time'])
            left_value, right_value = left[value_key], right[value_key]
            left_derivative, right_derivative = left[derivative_key], right[derivative_key]
            if dt <= 0.0 or not all(np.all(np.isfinite(value)) for value in (
                left_value, right_value, left_derivative, right_derivative,
            )):
                continue
            predicted_delta = 0.5 * (left_derivative + right_derivative) * dt
            residuals.append(np.linalg.norm((right_value - left_value) - predicted_delta))
        return np.asarray(residuals, dtype=float)

    def compute_trajectory_generation_metrics(
        self,
        trajectories: List[Dict],
        waypoint_positions: np.ndarray,
    ) -> List[MetricResult]:
        """从按轨迹身份去重后的 PlannerOutput 计算通用生成质量指标。"""
        definitions = (
            ('generated_trajectory_duration', '生成轨迹时长', 's'),
            ('waypoint_distance_mean', 'Waypoint 到生成轨迹平均距离', 'm'),
            ('waypoint_distance_p95', 'Waypoint 到生成轨迹距离 P95', 'm'),
            ('waypoint_distance_max', 'Waypoint 到生成轨迹最大距离', 'm'),
            ('speed_max', '生成轨迹最大速度', 'm/s'),
            ('acceleration_max', '生成轨迹最大加速度', 'm/s^2'),
            ('jerk_rms', '生成轨迹 Jerk RMS', 'm/s^3'),
            ('position_continuity_jump_max', '位置最大连续性跳变', 'm'),
            ('velocity_continuity_jump_max', '速度最大连续性跳变', 'm/s'),
            ('acceleration_continuity_jump_max', '加速度最大连续性跳变', 'm/s^2'),
        )
        source = '/planner/output (deduplicated by trajectory_id + desired time)'
        trajectory = self.select_primary_trajectory(trajectories)
        if trajectory is None:
            return [MetricResult(name=name, description=description, value=float('nan'),
                                 unit=unit, group='trajectory_quality', source=source,
                                 detail={'error': '没有包含至少两个有效位置点的 PlannerOutput 轨迹'})
                    for name, description, unit in definitions]

        points = trajectory['points']
        times = np.asarray([point['time'] for point in points], dtype=float)
        positions = np.asarray([point['position'] for point in points], dtype=float)
        trajectory_detail = {
            'trajectory_id': int(trajectory['trajectory_id']),
            'trajectory_count': int(len(trajectories)),
            'deduplicated_point_count': int(len(points)),
            'is_horizon': bool(trajectory.get('is_horizon', False)),
            'desired_time_start': float(times[0]),
            'desired_time_end': float(times[-1]),
            'time_from_start_min': float(min(point['time_from_start'] for point in points)),
            'time_from_start_max': float(max(point['time_from_start'] for point in points)),
        }
        duration = float(times[-1] - times[0])
        metrics = [MetricResult(
            name='generated_trajectory_duration', description='生成轨迹时长',
            value=duration, unit='s', group='trajectory_quality', source=source,
            detail=dict(trajectory_detail),
        )]

        waypoint_distances = self._point_to_polyline_distances(
            np.asarray(waypoint_positions, dtype=float), positions,
        )
        waypoint_values = (
            float(np.mean(waypoint_distances)) if len(waypoint_distances) else float('nan'),
            float(np.percentile(waypoint_distances, 95)) if len(waypoint_distances) else float('nan'),
            float(np.max(waypoint_distances)) if len(waypoint_distances) else float('nan'),
        )
        for (name, description), value in zip((
            ('waypoint_distance_mean', 'Waypoint 到生成轨迹平均距离'),
            ('waypoint_distance_p95', 'Waypoint 到生成轨迹距离 P95'),
            ('waypoint_distance_max', 'Waypoint 到生成轨迹最大距离'),
        ), waypoint_values):
            detail = dict(trajectory_detail)
            detail.update({
                'distance_definition': 'minimum Euclidean distance to continuous trajectory polyline',
                'waypoint_count': int(len(waypoint_positions)),
            })
            if not len(waypoint_distances):
                detail['error'] = 'waypoint 或生成轨迹折线不足'
            metrics.append(MetricResult(name=name, description=description, value=value,
                                        unit='m', group='trajectory_quality', source=source,
                                        detail=detail))

        for key, mask, name, description, unit, aggregation in (
            ('velocity', 2, 'speed_max', '生成轨迹最大速度', 'm/s', 'max'),
            ('acceleration', 4, 'acceleration_max', '生成轨迹最大加速度', 'm/s^2', 'max'),
            ('jerk', 8, 'jerk_rms', '生成轨迹 Jerk RMS', 'm/s^3', 'rms'),
        ):
            values = np.asarray([
                point[key] for point in points
                if int(point['valid_mask']) & mask and np.all(np.isfinite(point[key]))
            ], dtype=float)
            norms = np.linalg.norm(values, axis=1) if len(values) else np.empty(0)
            value = (
                float(np.max(norms)) if len(norms) and aggregation == 'max'
                else float(np.sqrt(np.mean(norms ** 2))) if len(norms)
                else float('nan')
            )
            detail = dict(trajectory_detail)
            detail['valid_sample_count'] = int(len(norms))
            if not len(norms):
                detail['error'] = f'PlannerOutput 未提供有效 {key}'
            metrics.append(MetricResult(name=name, description=description, value=value,
                                        unit=unit, group='trajectory_quality', source=source,
                                        detail=detail))

        for value_key, derivative_key, name, description, unit in (
            ('position', 'velocity', 'position_continuity_jump_max', '位置最大连续性跳变', 'm'),
            ('velocity', 'acceleration', 'velocity_continuity_jump_max', '速度最大连续性跳变', 'm/s'),
            ('acceleration', 'jerk', 'acceleration_continuity_jump_max', '加速度最大连续性跳变', 'm/s^2'),
        ):
            residuals = self._continuity_residual(points, value_key, derivative_key)
            detail = dict(trajectory_detail)
            detail.update({
                'definition': f'max trapezoidal integration residual of {value_key} from {derivative_key}',
                'valid_interval_count': int(len(residuals)),
            })
            if not len(residuals):
                detail['error'] = f'{value_key}/{derivative_key} 有效相邻采样不足'
            metrics.append(MetricResult(
                name=name, description=description,
                value=float(np.max(residuals)) if len(residuals) else float('nan'),
                unit=unit, group='trajectory_quality', source=source, detail=detail,
            ))
        return metrics

    def compute_stream_rate(
        self,
        times: np.ndarray,
        name: str = 'stream_rate',
        description: str = '输入流发布频率',
        group: str = 'interface_health',
        source: str = '/planner/output',
    ) -> MetricResult:
        """计算任意时间序列的发布频率。"""
        if len(times) < 2:
            return MetricResult(
                name=name,
                description=description,
                value=float('nan'),
                unit='Hz',
                group=group,
                source=source,
                detail={'error': '数据点不足'}
            )

        duration = float(times[-1] - times[0])
        rate = float((len(times) - 1) / duration) if duration > 0 else 0.0
        return MetricResult(
            name=name,
            description=description,
            value=rate,
            unit='Hz',
            group=group,
            source=source,
            detail={
                'sample_count': int(len(times)),
                'duration': duration,
            }
        )

    def compute_message_count(
        self,
        count: int,
        name: str,
        description: str,
        group: str = 'interface_health',
        source: str = '/planner/output',
    ) -> MetricResult:
        """记录某类消息数量。"""
        return MetricResult(
            name=name,
            description=description,
            value=float(count),
            unit='count',
            group=group,
            source=source,
            detail={'count': int(count)}
        )

    @staticmethod
    def compute_emergency_status(
        emergency_occurred: bool,
        emergency_time: float = 0.0
    ) -> MetricResult:
        """记录是否触发紧急状态。"""
        return MetricResult(
            name='emergency_status',
            description='紧急状态触发',
            value=1.0 if emergency_occurred else 0.0,
            unit='bool',
            group='outcome',
            source='runner metadata + recorded task state',
            detail={
                'triggered': bool(emergency_occurred),
                'trigger_time': float(emergency_time) if emergency_occurred else None,
            }
        )

    def compute_position_jitter_rms(
        self,
        positions: np.ndarray,
        reference: Optional[np.ndarray] = None
    ) -> MetricResult:
        """计算位置抖动 RMS。"""
        if len(positions) < 10:
            return MetricResult(
                name='position_jitter_rms',
                description='位置抖动 RMS',
                value=float('nan'),
                unit='m',
                group='stability',
                source='/mavros/local_position/odom',
                detail={'error': '数据点不足'}
            )

        mean_position = np.mean(positions, axis=0)
        deviations = positions - mean_position
        rms_x = np.sqrt(np.mean(deviations[:, 0] ** 2))
        rms_y = np.sqrt(np.mean(deviations[:, 1] ** 2))
        rms_z = np.sqrt(np.mean(deviations[:, 2] ** 2))
        rms_3d = np.sqrt(np.mean(np.sum(deviations ** 2, axis=1)))

        return MetricResult(
            name='position_jitter_rms',
            description='位置抖动 RMS',
            value=rms_3d,
            unit='m',
            group='stability',
            source='/mavros/local_position/odom',
            detail={
                'rms_x': float(rms_x),
                'rms_y': float(rms_y),
                'rms_z': float(rms_z),
                'rms_3d': float(rms_3d),
                'max_deviation_x': float(np.max(np.abs(deviations[:, 0]))),
                'max_deviation_y': float(np.max(np.abs(deviations[:, 1]))),
                'max_deviation_z': float(np.max(np.abs(deviations[:, 2]))),
                'max_deviation_3d': float(np.max(np.linalg.norm(deviations, axis=1))),
                'mean_position': mean_position.tolist(),
                'reference': reference.tolist() if isinstance(reference, np.ndarray) else reference,
            }
        )

    def compute_position_bias(
        self, positions: np.ndarray, reference: np.ndarray
    ) -> MetricResult:
        """计算稳态平均位置相对任务目标的三维偏差。"""
        if len(positions) < 10:
            return MetricResult(
                name='position_bias', description='稳态位置偏差',
                value=float('nan'), unit='m', group='tracking',
                source='odometry + task target',
                detail={'error': '数据点不足'},
            )
        bias = np.mean(positions, axis=0) - reference
        return MetricResult(
            name='position_bias', description='稳态位置偏差',
            value=float(np.linalg.norm(bias)), unit='m', group='tracking',
            source='odometry + task target',
            detail={
                'bias_x': float(bias[0]), 'bias_y': float(bias[1]),
                'bias_z': float(bias[2]), 'reference': reference.tolist(),
            },
        )

    def compute_z_steady_state_error(
        self,
        z_positions: np.ndarray,
        target_z: Optional[float] = None
    ) -> MetricResult:
        """计算 Z 轴稳态误差。"""
        if len(z_positions) < 10:
            return MetricResult(
                name='z_steady_state_error',
                description='Z 轴稳态误差',
                value=float('nan'),
                unit='m',
                group='tracking',
                source='odometry + task target height',
                detail={'error': '数据点不足'}
            )

        if target_z is None:
            target_z = self.hover_height

        errors = z_positions - target_z
        mean_error = float(np.mean(errors))
        std_error = float(np.std(errors))
        max_error = float(np.max(np.abs(errors)))

        return MetricResult(
            name='z_steady_state_error',
            description='Z 轴稳态误差',
            value=mean_error,
            unit='m',
            group='tracking',
            source='odometry + task target height',
            detail={
                'mean_error': mean_error,
                'std_error': std_error,
                'max_error': max_error,
                'target_z': target_z,
                'actual_mean_z': float(np.mean(z_positions)),
                'p95_error': float(np.percentile(np.abs(errors), 95)),
            }
        )

    def compute_attitude_fluctuation_rms(
        self,
        attitudes: np.ndarray
    ) -> MetricResult:
        """计算姿态波动 RMS。"""
        if len(attitudes) < 10:
            return MetricResult(
                name='attitude_fluctuation_rms',
                description='姿态波动 RMS',
                value=float('nan'),
                unit='rad',
                group='stability',
                source='/mavros/local_position/odom',
                detail={'error': '数据点不足'}
            )

        mean_att = np.mean(attitudes, axis=0)
        deviations = attitudes - mean_att

        rms_roll = float(np.sqrt(np.mean(deviations[:, 0] ** 2)))
        rms_pitch = float(np.sqrt(np.mean(deviations[:, 1] ** 2)))
        rms_yaw = float(np.sqrt(np.mean(deviations[:, 2] ** 2)))
        rms_total = float(np.sqrt(np.mean(np.sum(deviations ** 2, axis=1))))

        return MetricResult(
            name='attitude_fluctuation_rms',
            description='姿态波动 RMS',
            value=rms_total,
            unit='rad',
            group='stability',
            source='/mavros/local_position/odom',
            detail={
                'rms_roll_rad': rms_roll,
                'rms_pitch_rad': rms_pitch,
                'rms_yaw_rad': rms_yaw,
                'rms_roll_deg': float(math.degrees(rms_roll)),
                'rms_pitch_deg': float(math.degrees(rms_pitch)),
                'rms_yaw_deg': float(math.degrees(rms_yaw)),
                'max_roll_deg': float(math.degrees(float(np.max(np.abs(deviations[:, 0]))))),
                'max_pitch_deg': float(math.degrees(float(np.max(np.abs(deviations[:, 1]))))),
            }
        )

    def compute_roll_pitch_fluctuation_rms(
        self,
        attitudes: np.ndarray
    ) -> MetricResult:
        """计算仅包含 roll/pitch 的姿态波动 RMS。"""
        if len(attitudes) < 10:
            return MetricResult(
                name='roll_pitch_fluctuation_rms',
                description='Roll/Pitch 波动 RMS',
                value=float('nan'),
                unit='rad',
                group='diagnostic',
                source='/mavros/local_position/odom',
                detail={'error': '数据点不足'}
            )

        mean_att = np.mean(attitudes[:, :2], axis=0)
        deviations = attitudes[:, :2] - mean_att

        rms_roll = float(np.sqrt(np.mean(deviations[:, 0] ** 2)))
        rms_pitch = float(np.sqrt(np.mean(deviations[:, 1] ** 2)))
        rms_total = float(np.sqrt(np.mean(np.sum(deviations ** 2, axis=1))))
        return MetricResult(
            name='roll_pitch_fluctuation_rms',
            description='Roll/Pitch 波动 RMS',
            value=rms_total,
            unit='rad',
            group='diagnostic',
            source='/mavros/local_position/odom',
            detail={
                'rms_roll_rad': rms_roll,
                'rms_pitch_rad': rms_pitch,
                'rms_roll_deg': float(math.degrees(rms_roll)),
                'rms_pitch_deg': float(math.degrees(rms_pitch)),
                'max_roll_deg': float(math.degrees(float(np.max(np.abs(deviations[:, 0]))))),
                'max_pitch_deg': float(math.degrees(float(np.max(np.abs(deviations[:, 1]))))),
            }
        )

    def compute_yaw_tracking_rmse(
        self,
        attitudes: np.ndarray,
        attitude_times: np.ndarray,
        ref_pos_times: np.ndarray,
        reference_yaws: np.ndarray,
    ) -> MetricResult:
        """仅使用 VALID_YAW 显式声明的参考计算 yaw 跟踪 RMSE。"""
        if len(attitudes) < 10 or len(reference_yaws) < 2:
            return MetricResult(
                name='yaw_tracking_rmse',
                description='偏航跟踪误差 RMSE',
                value=float('nan'),
                unit='rad',
                group='tracking',
                source='odometry + explicit PlannerOutput yaw',
                detail={'error': '姿态或显式 VALID_YAW 参考数据点不足'}
            )
        actual, reference, _ = self._align_reference_samples(
            attitudes[:, 2:3], attitude_times,
            np.asarray(reference_yaws, dtype=float).reshape(-1, 1), ref_pos_times,
        )
        if len(actual) < 10:
            return MetricResult(
                name='yaw_tracking_rmse',
                description='偏航跟踪误差 RMSE',
                value=float('nan'),
                unit='rad',
                group='tracking',
                source='odometry + explicit PlannerOutput yaw',
                detail={'error': '姿态与有效显式 yaw 参考重叠不足'}
            )
        yaw_error = self._wrap_angle(actual[:, 0] - reference[:, 0])
        rmse = float(np.sqrt(np.mean(yaw_error ** 2)))

        return MetricResult(
            name='yaw_tracking_rmse',
            description='偏航跟踪误差 RMSE',
            value=rmse,
            unit='rad',
            group='tracking',
            source='odometry + explicit PlannerOutput yaw',
            detail={
                'mean_error_rad': float(np.mean(yaw_error)),
                'mean_error_deg': float(math.degrees(float(np.mean(yaw_error)))),
                'max_abs_error_rad': float(np.max(np.abs(yaw_error))),
                'max_abs_error_deg': float(math.degrees(float(np.max(np.abs(yaw_error))))),
                'p95_error_deg': float(math.degrees(float(np.percentile(np.abs(yaw_error), 95)))),
                'sample_count': int(len(yaw_error)),
                'reference_requirement': 'PlannerOutputPoint.VALID_YAW',
            }
        )

    def compute_thrust_std(
        self, thrusts: np.ndarray, group: str = 'diagnostic',
    ) -> MetricResult:
        if len(thrusts) < 10:
            return MetricResult(
                name='thrust_std', description='推力标准差',
                value=float('nan'), unit='', group=group,
                source='controller thrust command', detail={'error': '数据点不足'},
            )
        return MetricResult(
            name='thrust_std', description='推力标准差',
            value=float(np.std(thrusts)), unit='', group=group,
            source='controller thrust command',
            detail={
                'mean_thrust': float(np.mean(thrusts)),
                'min_thrust': float(np.min(thrusts)),
                'max_thrust': float(np.max(thrusts)),
                'sample_count': int(len(thrusts)),
            },
        )

    def detect_oscillation(
        self,
        signal: np.ndarray,
        sample_rate: float = 100.0,
        min_freq: float = 0.5,
        max_freq: float = 20.0,
        name_suffix: str = '',
        times: Optional[np.ndarray] = None,
        min_amplitude: float = 0.0,
    ) -> MetricResult:
        """检测振荡；非均匀采样会先重采样，再去线性趋势并加 Hann 窗。"""
        result = self._detect_oscillation_impl(
            signal, sample_rate, min_freq, max_freq,
            times=times, min_amplitude=min_amplitude,
        )
        result.name = f'oscillation{name_suffix}'
        return result

    def _detect_oscillation_impl(
        self,
        signal: np.ndarray,
        sample_rate: float = 100.0,
        min_freq: float = 0.5,
        max_freq: float = 20.0,
        times: Optional[np.ndarray] = None,
        min_amplitude: float = 0.0,
    ) -> MetricResult:
        """振荡检测实现"""
        if len(signal) < 64:
            return MetricResult(
                name='oscillation_detection',
                description='振荡检测',
                value=float('nan'),
                unit='',
                group='stability',
                source='derived from recorded signal spectrum',
                detail={'error': '数据点不足 (需要至少64个采样点)'}
            )

        signal = np.asarray(signal, dtype=float)
        if times is not None:
            times = np.asarray(times, dtype=float)
            valid = np.isfinite(times) & np.isfinite(signal)
            times, signal = times[valid], signal[valid]
            order = np.argsort(times)
            times, signal = times[order], signal[order]
            times, unique_idx = np.unique(times, return_index=True)
            signal = signal[unique_idx]
            if len(times) < 64 or times[-1] <= times[0]:
                return MetricResult(
                    name='oscillation_detection', description='振荡检测',
                    value=float('nan'), unit='', group='stability',
                    source='derived from recorded signal spectrum',
                    detail={'error': '有效时间序列数据不足'}
                )
            dt = float(np.median(np.diff(times)))
            if dt <= 0:
                dt = 1.0 / sample_rate
            sample_rate = 1.0 / dt
            uniform_times = np.arange(times[0], times[-1] + 0.5 * dt, dt)
            signal = np.interp(uniform_times, times, signal)

        n = len(signal)
        x = np.arange(n, dtype=float)
        trend = np.polyval(np.polyfit(x, signal, 1), x)
        detrended = signal - trend
        window = np.hanning(n)
        windowed = detrended * window
        fft_vals = np.fft.rfft(windowed)
        freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate)
        power = np.abs(fft_vals) ** 2
        coherent_gain = max(float(np.sum(window)), 1.0)
        amplitudes = 2.0 * np.abs(fft_vals) / coherent_gain

        freq_mask = (freqs >= min_freq) & (freqs <= max_freq)
        if not np.any(freq_mask):
            return MetricResult(
                name='oscillation_detection',
                description='振荡检测',
                value=0.0,
                unit='',
                group='stability',
                source='derived from recorded signal spectrum',
                detail={'has_oscillation': False, 'reason': '频率范围无数据'}
            )

        power_band = power[freq_mask]
        freqs_band = freqs[freq_mask]
        amplitudes_band = amplitudes[freq_mask]

        noise_floor = np.median(power_band)
        peak_threshold = noise_floor * 3.0
        peak_indices = np.where(
            (power_band > peak_threshold) &
            (amplitudes_band >= min_amplitude)
        )[0]

        has_oscillation = len(peak_indices) > 0

        if has_oscillation:
            dominant_idx = peak_indices[np.argmax(power_band[peak_indices])]
            dominant_freq = float(freqs_band[dominant_idx])
            dominant_power = float(power_band[dominant_idx])
            dominant_amplitude = float(amplitudes_band[dominant_idx])
            snr = dominant_power / noise_floor if noise_floor > 0 else float('inf')
        else:
            dominant_freq = 0.0
            dominant_power = 0.0
            dominant_amplitude = 0.0
            snr = 0.0

        return MetricResult(
            name='oscillation_detection',
            description='振荡检测',
            value=dominant_freq,
            unit='Hz',
            group='stability',
            source='derived from recorded signal spectrum',
            detail={
                'has_oscillation': has_oscillation,
                'dominant_freq_hz': dominant_freq,
                'dominant_power': dominant_power,
                'dominant_amplitude': dominant_amplitude,
                'snr': float(snr),
                'noise_floor': float(noise_floor),
                'num_peaks': int(len(peak_indices)),
                'sample_rate_hz': float(sample_rate),
                'min_amplitude': float(min_amplitude),
                'preprocessing': 'linear_detrend+hann',
            }
        )
