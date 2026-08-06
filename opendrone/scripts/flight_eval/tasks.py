#!/usr/bin/env python3
"""Flight evaluation task definitions."""

import abc
import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from .outcomes import (
    MISSION_GOAL_DWELL_TIME,
    MISSION_GOAL_TOLERANCE,
    MISSION_PATH_TOPIC,
    NoOutcomeEvaluator,
    PathGoalEvaluator,
    TRAJECTORY_TRIGGER_TOPIC,
)


@dataclass
class FlightPhase:
    name: str
    state_code: int
    analysis_type: str = 'ignore'


class TaskBase(abc.ABC):
    """任务基类"""

    def __init__(self, takeoff_height: float = 2.0, duration: Optional[float] = None):
        self.takeoff_height = takeoff_height
        self._duration = float(self.default_duration if duration is None else duration)

    @property
    @abc.abstractmethod
    def name(self) -> str:
        pass

    @property
    @abc.abstractmethod
    def description(self) -> str:
        pass

    @property
    def flight_phases(self) -> List[FlightPhase]:
        return [
            FlightPhase('waiting_connected', 0),
            FlightPhase('waiting_offboard', 1),
            FlightPhase('takeoff', 2, 'transient'),
            FlightPhase('mission', 3, 'evaluation'),
            FlightPhase('landing', 4),
            FlightPhase('landed', 5),
            FlightPhase('emergency', 6),
        ]

    @abc.abstractmethod
    def get_hover_height(self) -> float:
        """返回悬停目标高度"""
        pass

    @property
    def target_state_code(self) -> int:
        for phase in self.flight_phases:
            if phase.analysis_type == 'evaluation':
                return phase.state_code
        return 3

    @property
    def duration(self) -> float:
        """定时任务的评价窗口，或有限终点任务的完成期限。"""
        return self._duration

    @property
    def default_duration(self) -> float:
        """任务默认主阶段时长。"""
        return 20.0

    @property
    def metric_profile(self) -> str:
        """稳定的指标集合标识；只描述任务语义，不表示评分或报告视图。"""
        return 'online_mission'

    @property
    def render_trajectory_plots(self) -> bool:
        """是否生成 2D/3D 空间轨迹图。"""
        return True

    @property
    def execution_start_policy(self) -> str:
        """任务执行起点策略。"""
        return 'mission_state'

    @property
    def waypoint_type(self) -> str:
        """传给 waypoint_generator 的预设航点类型"""
        return ''

    @property
    def planner_name_override(self) -> str:
        """任务要求的参考源 / planner 启动项。空字符串表示不覆盖。"""
        return ''

    @property
    def planner_args(self) -> Dict[str, str]:
        """任务要求注入的 planner launch 参数。"""
        return {}

    @property
    def record_topics(self) -> List[str]:
        """任务协议自身拥有的录包话题。"""
        return []

    @property
    def has_terminal_outcome(self) -> bool:
        """任务是否存在可提前结束执行阶段的语义终点。"""
        return False

    def create_outcome_evaluator(self):
        """创建一次运行独占的任务结果判定器。"""
        return NoOutcomeEvaluator()

    def get_target_position(self) -> np.ndarray:
        """返回任务目标位置。默认任务是在世界坐标原点上方悬停。"""
        return np.array([0.0, 0.0, self.get_hover_height()])

    def get_analysis_phase(self, analysis_type: str = 'evaluation') -> FlightPhase:
        for phase in self.flight_phases:
            if phase.analysis_type == analysis_type:
                return phase
        raise ValueError(f"任务 {self.name} 未定义 {analysis_type} 分析阶段")

    @staticmethod
    def compute_tracking_oscillation_metric(
        calculator, positions: np.ndarray, position_times: np.ndarray,
        refs: np.ndarray, ref_times: np.ndarray,
    ):
        error_norm = np.empty(0)
        aligned_times = np.empty(0)
        if len(positions) > 64 and len(refs) > 2:
            aligned_positions, aligned_refs, aligned_times = calculator._align_reference_samples(
                positions, position_times, refs, ref_times,
            )
            if len(aligned_positions) > 64:
                error_norm = np.linalg.norm(aligned_positions - aligned_refs, axis=1)
        metric = calculator.detect_oscillation(
            error_norm,
            HoverTask._estimate_sample_rate(aligned_times),
            times=aligned_times,
            min_amplitude=0.01,
        )
        metric.name = 'tracking_error_spectrum_peak'
        metric.description = '跟踪误差频谱峰值'
        metric.group = 'diagnostic'
        metric.source = 'position tracking error spectrum'
        return metric

    @classmethod
    def compute_explicit_tracking_metrics(
        cls, calculator, phase_data: Dict, tracking_group: str = 'tracking',
    ) -> List:
        positions = phase_data.get('positions', np.empty((0, 3)))
        position_times = phase_data.get('position_times', np.empty(0))
        velocities = phase_data.get('velocities', np.empty((0, 3)))
        velocity_times = phase_data.get('velocity_times', np.empty(0))
        attitudes = phase_data.get('attitudes', np.empty((0, 3)))
        attitude_times = phase_data.get('attitude_times', np.empty(0))
        thrusts = phase_data.get('thrusts', np.empty(0))
        refs = phase_data.get('reference_positions', np.empty((0, 3)))
        ref_velocities = phase_data.get('reference_velocities', np.empty((0, 3)))
        ref_times = phase_data.get('ref_pos_times', np.empty(0))
        ref_yaws = phase_data.get('reference_yaws', np.empty(0))

        primary = calculator.compute_position_tracking_metrics(
            positions, position_times, refs, ref_times,
        )
        primary.append(calculator.compute_velocity_tracking_error(
            velocities, velocity_times, ref_velocities, ref_times,
        ))
        if len(ref_yaws) and np.any(np.isfinite(ref_yaws)):
            primary.append(calculator.compute_yaw_tracking_rmse(
                attitudes, attitude_times, ref_times, ref_yaws,
            ))
        for metric in primary:
            metric.group = tracking_group

        spatial = calculator.compute_spatial_shape_metrics(
            positions, position_times, refs, ref_times,
        )
        diagnostics = calculator.compute_reference_availability(position_times, ref_times)
        roll_pitch = calculator.compute_roll_pitch_fluctuation_rms(attitudes)
        roll_pitch.group = 'diagnostic'
        diagnostics.append(roll_pitch)
        diagnostics.append(calculator.compute_thrust_std(thrusts))
        diagnostics.append(cls.compute_tracking_oscillation_metric(
            calculator, positions, position_times, refs, ref_times,
        ))
        return primary + spatial + diagnostics

    @abc.abstractmethod
    def compute_metrics(self, calculator, phase_data: Dict, full_data: Dict) -> List:
        """根据任务语义计算指标"""
        pass


class HoverTask(TaskBase):
    """Controller hover task."""

    def __init__(self, takeoff_height: float = 2.0, duration: Optional[float] = None):
        super().__init__(takeoff_height, duration)

    @property
    def name(self) -> str:
        return 'hover'

    @property
    def description(self) -> str:
        return f'起飞悬停任务: 起飞到 {self.takeoff_height}m, 悬停 {self.duration}s'

    @property
    def flight_phases(self) -> List[FlightPhase]:
        return [
            FlightPhase('waiting_connected', 0),
            FlightPhase('waiting_offboard', 1),
            FlightPhase('takeoff', 2, 'transient'),
            FlightPhase('hover', 3, 'evaluation'),
            FlightPhase('landing', 4),
            FlightPhase('landed', 5),
            FlightPhase('emergency', 6),
        ]

    def get_hover_height(self) -> float:
        return self.takeoff_height

    @property
    def metric_profile(self) -> str:
        return 'hover_regulation'

    @property
    def default_duration(self) -> float:
        return 15.0

    @property
    def render_trajectory_plots(self) -> bool:
        return False

    def compute_metrics(self, calculator, phase_data: Dict, full_data: Dict) -> List:
        """计算自动起飞悬停任务的稳态指标"""
        metrics = []
        positions = phase_data.get('positions', np.empty((0, 3)))
        attitudes = phase_data.get('attitudes', np.empty((0, 3)))
        attitude_times = phase_data.get('attitude_times', np.empty(0))
        thrusts = phase_data.get('thrusts', np.empty(0))
        thrust_times = phase_data.get('thrust_times', np.empty(0))
        position_times = phase_data.get('position_times', np.empty(0))

        metrics.append(calculator.compute_position_bias(
            positions, reference=self.get_target_position(),
        ))
        metrics.append(calculator.compute_position_jitter_rms(
            positions, reference=self.get_target_position(),
        ))
        metrics.append(calculator.compute_z_steady_state_error(
            positions[:, 2], target_z=self.get_hover_height(),
        ))
        metrics.append(calculator.compute_attitude_fluctuation_rms(attitudes))
        metrics.append(calculator.compute_thrust_std(thrusts, group='stability'))
        metrics.append(calculator.detect_oscillation(
            positions[:, 0], name_suffix='_pos_x', times=position_times,
            min_amplitude=0.01,
        ))
        metrics.append(calculator.detect_oscillation(
            positions[:, 2], name_suffix='_pos_z', times=position_times,
            min_amplitude=0.01,
        ))
        metrics.append(calculator.detect_oscillation(
            attitudes[:, 0], name_suffix='_roll', times=attitude_times,
            min_amplitude=math.radians(0.25),
        ))
        metrics.append(calculator.detect_oscillation(
            thrusts, name_suffix='_thrust', times=thrust_times,
            min_amplitude=0.005,
        ))

        return metrics

    @staticmethod
    def _estimate_sample_rate(times: np.ndarray, default: float = 100.0) -> float:
        if len(times) <= 1:
            return default

        dt = np.median(np.diff(times))
        if dt <= 0:
            return default
        return float(1.0 / dt)


class PresetTrajectoryTask(TaskBase):
    """预设航点轨迹任务基类"""

    task_name = ''
    preset_name = ''
    display_name = ''

    @property
    def name(self) -> str:
        return self.task_name

    @property
    def waypoint_type(self) -> str:
        return self.preset_name

    @property
    def description(self) -> str:
        return f'{self.display_name}轨迹任务: 起飞到 {self.takeoff_height}m 后执行 {self.duration}s'

    def get_hover_height(self) -> float:
        return self.takeoff_height

    @property
    def metric_profile(self) -> str:
        return 'trajectory_generation'

    @property
    def default_duration(self) -> float:
        return 100.0

    @property
    def execution_start_policy(self) -> str:
        return 'first_task_signal'

    @property
    def planner_args(self) -> Dict[str, str]:
        return {
            'use_preset_waypoints': 'true',
            'auto_trigger_waypoints': 'true',
        }

    @property
    def record_topics(self) -> List[str]:
        return [MISSION_PATH_TOPIC, TRAJECTORY_TRIGGER_TOPIC]

    def compute_metrics(self, calculator, phase_data: Dict, full_data: Dict) -> List:
        """根据任务语义计算预设轨迹任务指标。"""
        metrics = []
        planner_message_times = phase_data.get('planner_output_message_times', np.empty(0))
        waypoint_positions = full_data.get('waypoint_positions', np.empty((0, 3)))
        trajectories = phase_data.get('planner_trajectories', [])
        primary_trajectory = calculator.select_primary_trajectory(trajectories)
        planner_positions = (
            np.asarray([point['position'] for point in primary_trajectory['points']], dtype=float)
            if primary_trajectory is not None else np.empty((0, 3))
        )

        metrics.extend(calculator.compute_trajectory_generation_metrics(
            trajectories, waypoint_positions,
        ))
        metrics.append(calculator.compute_stream_rate(
            planner_message_times,
            name='planner_output_rate',
            description='规划输出发布频率',
        ))
        metrics.append(calculator.compute_message_count(
            int(phase_data.get('planner_output_message_count', 0)),
            'planner_output_message_count',
            '/planner/output 消息数量',
        ))
        metrics.append(calculator.compute_message_count(
            int(phase_data.get('planner_output_point_count', 0)),
            'planner_output_point_count',
            '/planner/output 采样点数量',
        ))

        metrics.extend(self.compute_explicit_tracking_metrics(
            calculator, phase_data, tracking_group='execution_tracking',
        ))

        metrics.extend(self.compute_geometry_metrics(planner_positions, full_data))
        return metrics

    def compute_geometry_metrics(self, positions: np.ndarray, full_data: Dict) -> List:
        return []


class PlanMissionTask(TaskBase):
    """Integrated obstacle-environment mission."""

    @property
    def name(self) -> str:
        return 'plan_mission'

    @property
    def description(self) -> str:
        return (
            f'障碍环境整链路任务: 起飞到 {self.takeoff_height}m 后在 '
            f'{self.duration}s 期限内到达整体终点'
        )

    def get_hover_height(self) -> float:
        return self.takeoff_height

    @property
    def metric_profile(self) -> str:
        return 'online_mission'

    @property
    def default_duration(self) -> float:
        return 120.0

    @property
    def execution_start_policy(self) -> str:
        return 'first_task_signal'

    @property
    def waypoint_type(self) -> str:
        return 'manual'

    @property
    def planner_args(self) -> Dict[str, str]:
        """整链路任务必须主动发起预设 mission，不能依赖人工点击触发。"""
        return {
            'use_preset_waypoints': 'true',
            'auto_trigger_waypoints': 'true',
        }

    @property
    def record_topics(self) -> List[str]:
        return [MISSION_PATH_TOPIC, TRAJECTORY_TRIGGER_TOPIC]

    @property
    def has_terminal_outcome(self) -> bool:
        return True

    def create_outcome_evaluator(self):
        return PathGoalEvaluator(
            goal_tolerance=MISSION_GOAL_TOLERANCE,
            progress_tolerance=0.5,
            dwell_time=MISSION_GOAL_DWELL_TIME,
        )

    def compute_metrics(self, calculator, phase_data: Dict, full_data: Dict) -> List:
        """计算 mission 型整链路任务指标。"""
        metrics = []
        planner_message_times = phase_data.get('planner_output_message_times', np.empty(0))

        metrics.append(calculator.compute_stream_rate(
            planner_message_times,
            name='planner_output_rate',
            description='规划输出发布频率',
        ))
        metrics.append(calculator.compute_message_count(
            int(phase_data.get('planner_output_message_count', 0)),
            'planner_output_message_count',
            '/planner/output 消息数量',
        ))
        metrics.append(calculator.compute_message_count(
            int(phase_data.get('planner_output_point_count', 0)),
            'planner_output_point_count',
            '/planner/output 采样点数量',
        ))
        metrics.extend(self.compute_explicit_tracking_metrics(calculator, phase_data))

        return metrics


class AnalyticReferenceTaskBase(PresetTrajectoryTask):
    """Controller task driven by an analytic reference source."""

    trajectory_type = ''

    @property
    def description(self) -> str:
        return (
            f'{self.display_name}参考跟踪任务: 起飞到 {self.takeoff_height}m 后执行 '
            f'{self.duration}s'
        )

    @property
    def metric_profile(self) -> str:
        return 'trajectory_tracking'

    @property
    def planner_name_override(self) -> str:
        return 'analytic_reference'

    @property
    def planner_args(self) -> Dict[str, str]:
        return {
            'trajectory_type': self.trajectory_type,
            'center_z': str(self.takeoff_height),
        }

    @property
    def default_duration(self) -> float:
        return 40.0

    @property
    def execution_start_policy(self) -> str:
        return 'first_reference_or_planner_output'

    @property
    def record_topics(self) -> List[str]:
        return []

    def compute_metrics(self, calculator, phase_data: Dict, full_data: Dict) -> List:
        """Compute analytic-reference tracking metrics."""
        del full_data
        metrics = self.compute_explicit_tracking_metrics(calculator, phase_data)
        metrics.append(calculator.compute_message_count(
            int(phase_data.get('planner_output_message_count', 0)),
            'planner_output_message_count',
            '/planner/output 消息数量',
        ))

        return metrics


class DiscreteCircleTrajectoryTask(PresetTrajectoryTask):
    """离散圆点重建任务"""

    task_name = 'discrete_circle'
    preset_name = 'circle'
    display_name = '离散圆点'

    @property
    def default_duration(self) -> float:
        return 80.0

    @staticmethod
    def _estimate_circle_from_waypoints(
        waypoint_positions: np.ndarray,
    ) -> Optional[Dict[str, np.ndarray]]:
        if len(waypoint_positions) < 3:
            return None

        xy = waypoint_positions[:, :2]
        center = np.mean(xy, axis=0)
        radii = np.linalg.norm(xy - center, axis=1)
        radius = float(np.mean(radii))
        if radius <= 1e-6:
            return None

        return {
            'center': center,
            'radius': radius,
            'start_xy': xy[0],
        }

    @staticmethod
    def _compute_unwrapped_angles(xy: np.ndarray, center: np.ndarray) -> np.ndarray:
        angles = np.arctan2(xy[:, 1] - center[1], xy[:, 0] - center[0])
        return np.unwrap(angles)

    def _compute_waypoint_geometry_metrics(
        self,
        geometry_positions: np.ndarray,
        waypoint_positions: np.ndarray,
    ) -> List:
        from .metrics import MetricResult

        circle = self._estimate_circle_from_waypoints(waypoint_positions)
        if circle is None or len(geometry_positions) < 10:
            reason = 'waypoint 几何或轨迹采样点不足'
            return [
                MetricResult(name=name, description=description, value=float('nan'),
                             unit=unit, group='trajectory_quality',
                             source='/planner/output + mission waypoints',
                             detail={'error': reason, 'geometry_source': 'planner_output_only'})
                for name, description, unit in (
                    ('circle_angular_coverage', '圆轨迹角度覆盖率', 'turn'),
                    ('circle_radius_error_rmse', '圆轨迹半径误差 RMSE', 'm'),
                )
            ]

        xy = geometry_positions[:, :2]
        center = circle['center']
        target_radius = float(circle['radius'])
        radii = np.linalg.norm(xy - center, axis=1)
        radius_error = radii - target_radius
        radius_rmse = float(np.sqrt(np.mean(radius_error ** 2)))

        angles = self._compute_unwrapped_angles(xy, center)
        angular_coverage = float(np.ptp(angles) / (2.0 * np.pi))

        return [
            MetricResult(
                name='circle_angular_coverage',
                description='圆轨迹角度覆盖率',
                value=angular_coverage,
                unit='turn',
                group='trajectory_quality',
                source='/planner/output + mission waypoints',
                detail={
                    'coverage_deg': float(angular_coverage * 360.0),
                    'sample_count': int(len(geometry_positions)),
                    'geometry_source': 'planner_output_only',
                },
            ),
            MetricResult(
                name='circle_radius_error_rmse',
                description='圆轨迹半径误差 RMSE',
                value=radius_rmse,
                unit='m',
                group='trajectory_quality',
                source='/planner/output + mission waypoints',
                detail={
                    'target_radius': target_radius,
                    'mean_radius': float(np.mean(radii)),
                    'max_abs_radius_error': float(np.max(np.abs(radius_error))),
                    'geometry_source': 'planner_output_only',
                },
            ),
        ]

    def compute_geometry_metrics(self, positions: np.ndarray, full_data: Dict) -> List:
        waypoint_positions = full_data.get('waypoint_positions', np.empty((0, 3)))
        return self._compute_waypoint_geometry_metrics(positions, waypoint_positions)


class DiscreteFigure8TrajectoryTask(PresetTrajectoryTask):
    """离散 8 字点重建任务"""

    task_name = 'discrete_figure8'
    preset_name = 'eight'
    display_name = '离散8字点'

    @property
    def default_duration(self) -> float:
        return 110.0

    def compute_geometry_metrics(self, positions: np.ndarray, full_data: Dict) -> List:
        from .metrics import MetricResult

        del full_data
        source = positions
        if len(source) < 10:
            return [
                MetricResult(name=name, description=description, value=float('nan'),
                             unit='m', group='trajectory_quality',
                             source='/planner/output',
                             detail={'error': '轨迹采样点不足',
                                     'geometry_source': 'planner_output_only'})
                for name, description in (
                    ('figure8_closure_error', '8字轨迹闭合误差'),
                    ('figure8_crossing_error', '8字中心交叉误差'),
                )
            ]

        xy = source[:, :2]
        closure_error = float(np.linalg.norm(xy[-1] - xy[0]))
        center = np.mean(xy, axis=0)
        center_dist = np.linalg.norm(xy - center, axis=1)
        crossing_error = float(np.min(center_dist)) if len(center_dist) else float('nan')

        return [
            MetricResult(
                name='figure8_closure_error',
                description='8字轨迹闭合误差',
                value=closure_error,
                unit='m',
                group='trajectory_quality',
                source='/planner/output',
                detail={'geometry_source': 'planner_output_only'}
            ),
            MetricResult(
                name='figure8_crossing_error',
                description='8字中心交叉误差',
                value=crossing_error,
                unit='m',
                group='trajectory_quality',
                source='/planner/output',
                detail={
                    'estimated_center_x': float(center[0]),
                    'estimated_center_y': float(center[1]),
                    'geometry_source': 'planner_output_only',
                }
            ),
        ]


class AnalyticCircleTrajectoryTask(AnalyticReferenceTaskBase):
    """解析圆参考跟踪任务"""

    task_name = 'analytic_circle'
    display_name = '解析圆'
    trajectory_type = 'circle'

class AnalyticFigure8TrajectoryTask(AnalyticReferenceTaskBase):
    """解析 8 字参考跟踪任务"""

    task_name = 'analytic_figure8'
    display_name = '解析8字'
    trajectory_type = 'figure8'


class AnalyticSpiralTrajectoryTask(AnalyticReferenceTaskBase):
    """解析螺旋参考跟踪任务"""

    task_name = 'analytic_spiral'
    display_name = '解析螺旋'
    trajectory_type = 'spiral'


TASK_REGISTRY = {
    'hover': HoverTask,
    'plan_mission': PlanMissionTask,
    'discrete_circle': DiscreteCircleTrajectoryTask,
    'discrete_figure8': DiscreteFigure8TrajectoryTask,
    'analytic_circle': AnalyticCircleTrajectoryTask,
    'analytic_figure8': AnalyticFigure8TrajectoryTask,
    'analytic_spiral': AnalyticSpiralTrajectoryTask,
}


def create_task(task_name: str, **kwargs) -> TaskBase:
    """根据任务名创建任务实例"""
    if task_name not in TASK_REGISTRY:
        raise ValueError(f"未知任务: {task_name}, 可用任务: {list(TASK_REGISTRY.keys())}")
    return TASK_REGISTRY[task_name](**kwargs)
