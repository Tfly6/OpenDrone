#!/usr/bin/env python3
"""Render plots and flight replays from analyzed bag data."""

import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

from .analyzer import BagAnalyzer
from .metrics import AnalysisReport, MetricResult


PLOTS_DIRNAME = 'plots'


class BagVisualizer:
    """从 rosbag 生成静态论文图和可选的三维飞行回放 GIF。"""

    def __init__(
        self,
        bag_file: str,
        controller_name: str,
        task_name: str = 'hover',
        planner_name: str = 'none',
        hover_height: Optional[float] = None,
        duration: Optional[float] = None,
    ):
        self.analyzer = BagAnalyzer(
            bag_file=bag_file,
            controller_name=controller_name,
            task_name=task_name,
            planner_name=planner_name,
            hover_height=hover_height,
            duration=duration,
        )

    @staticmethod
    def _import_plotting():
        """延迟导入绘图库，避免普通 analyze 命令依赖 GUI 环境。"""
        try:
            os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.animation import PillowWriter
        except ImportError as exc:
            raise RuntimeError(
                '可视化需要 matplotlib 和 Pillow。请安装 python3-matplotlib python3-pil。'
            ) from exc

        plt.rcParams.update({
            'font.family': 'DejaVu Sans',
            'font.size': 10,
            'axes.labelsize': 10,
            'axes.titlesize': 12,
            'legend.fontsize': 8,
            'figure.dpi': 120,
            'savefig.dpi': 220,
            'axes.spines.top': False,
            'axes.spines.right': False,
        })
        return plt, PillowWriter

    @staticmethod
    def _series(
        data: Dict,
        values_key: str,
        times_key: str,
        start_time: Optional[float],
        end_time: Optional[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        values = data.get(values_key, np.empty(0))
        times = data.get(times_key, np.empty(0))
        if len(values) == 0 or len(times) == 0:
            return np.empty((0,) + tuple(np.shape(values)[1:])), np.empty(0)
        mask = np.ones(len(times), dtype=bool)
        if start_time is not None:
            mask &= times >= start_time
        if end_time is not None:
            mask &= times <= end_time
        return values[mask], times[mask]

    @staticmethod
    def _sort_by_time(values: np.ndarray, times: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if len(times) < 2:
            return values, times
        order = np.argsort(times, kind='stable')
        return values[order], times[order]

    @staticmethod
    def _set_equal_3d_axes(ax, points: np.ndarray):
        """保持 x/y/z 比例一致，防止轨迹在论文图中失真。"""
        mins = np.min(points, axis=0)
        maxs = np.max(points, axis=0)
        centers = (mins + maxs) / 2.0
        half_range = max(float(np.max(maxs - mins)) / 2.0, 0.5)
        ax.set_xlim(centers[0] - half_range, centers[0] + half_range)
        ax.set_ylim(centers[1] - half_range, centers[1] + half_range)
        ax.set_zlim(centers[2] - half_range, centers[2] + half_range)
        try:
            ax.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass

    @staticmethod
    def _format_metric(metric: MetricResult) -> str:
        if not metric.available:
            return 'N/A'
        if metric.name == 'emergency_status':
            return 'triggered' if metric.detail.get('triggered') else 'not triggered'
        return f'{metric.value:.4g} {metric.unit}'.strip()

    @staticmethod
    def _metric_label(metric: MetricResult) -> str:
        """论文图使用稳定的英文机器名，避免主机字体缺少中文 glyph。"""
        return metric.name.replace('_', ' ').title()

    @staticmethod
    def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """ENU 中的 ZYX 欧拉角旋转矩阵，用于绘制无人机机架。"""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    def _collect_data(self, scope: str) -> Tuple[Optional[AnalysisReport], Dict, Tuple[float, float]]:
        """分析 bag，并取得渲染所需的完整数据和任务 phase。"""
        report = self.analyzer.analyze()
        full_data, phase_data = self.analyzer.get_cached_data()
        start_time, end_time = self._resolve_time_range(full_data, phase_data, scope)
        return report, full_data, (start_time, end_time)

    @staticmethod
    def _resolve_time_range(
        full_data: Dict,
        phase_data: Dict,
        scope: str,
    ) -> Tuple[float, float]:
        if scope == 'execution':
            start_time = phase_data.get('start_time')
            end_time = phase_data.get('end_time')
        else:
            position_times = full_data.get('position_times', np.empty(0))
            if len(position_times) == 0:
                raise ValueError('bag 缺少 /mavros/local_position/odom，无法生成飞行可视化。')
            start_time, end_time = float(position_times[0]), float(position_times[-1])
        if start_time is None or end_time is None or end_time <= start_time:
            raise ValueError('没有有效的飞行时间区间可供可视化。')
        return float(start_time), float(end_time)

    def _build_plot_data(self, data: Dict, time_range: Tuple[float, float]) -> Dict:
        start_time, end_time = time_range
        actual, actual_t = self._series(data, 'positions', 'position_times', start_time, end_time)
        attitude, attitude_t = self._series(data, 'attitudes', 'attitude_times', start_time, end_time)
        thrust, thrust_t = self._series(data, 'thrusts', 'thrust_times', start_time, end_time)
        reference, reference_t = self._series(data, 'reference_positions', 'ref_pos_times', start_time, end_time)
        reference_yaws, reference_yaw_t = self._series(
            data, 'reference_yaws', 'ref_pos_times', start_time, end_time,
        )
        planner_segments = self._planner_segments(data, start_time, end_time)
        planner = (
            np.vstack(planner_segments)
            if planner_segments else np.empty((0, 3))
        )
        waypoints, _ = self._series(data, 'waypoint_positions', 'waypoint_times', start_time, end_time)

        reference, reference_t = self._sort_by_time(reference, reference_t)
        reference_yaws, reference_yaw_t = self._sort_by_time(reference_yaws, reference_yaw_t)
        return {
            'actual': actual, 'actual_t': actual_t,
            'attitude': attitude, 'attitude_t': attitude_t,
            'thrust': thrust, 'thrust_t': thrust_t,
            'reference': reference, 'reference_t': reference_t,
            'reference_yaws': reference_yaws, 'reference_yaw_t': reference_yaw_t,
            'planner': planner, 'planner_segments': planner_segments,
            'waypoints': waypoints,
            'start_time': start_time, 'end_time': end_time,
        }

    def _planner_segments(self, data: Dict, start_time: float, end_time: float) -> List[np.ndarray]:
        """按 PlannerOutput 消息窗口返回轨迹段，避免连接不同 horizon。"""
        segments = []
        for window in data.get('planner_output_windows', []):
            points = window.get('points', [])
            if not points:
                continue
            times = np.asarray([point['time'] for point in points], dtype=float)
            positions = np.asarray([point['position'] for point in points], dtype=float)
            mask = (times >= start_time) & (times <= end_time)
            if not np.any(mask):
                continue
            positions, times = self._sort_by_time(positions[mask], times[mask])
            segments.append(positions)

        return segments

    @staticmethod
    def _find_metric(report: Optional[AnalysisReport], name: str) -> Optional[MetricResult]:
        if report is None:
            return None
        return next((metric for metric in report.metrics if metric.name == name), None)

    @staticmethod
    def _tracking_alignment(plot_data: Dict) -> Optional[Dict]:
        """按指标对齐真实存在的参考样本，不跨 reference gap 插值。"""
        actual, actual_t = plot_data['actual'], plot_data['actual_t']
        reference, reference_t = plot_data['reference'], plot_data['reference_t']
        if len(actual) < 2 or len(reference) < 2:
            return None
        indices = np.searchsorted(reference_t, actual_t)
        right = np.clip(indices, 0, len(reference_t) - 1)
        left = np.clip(indices - 1, 0, len(reference_t) - 1)
        choose_left = np.abs(reference_t[left] - actual_t) <= np.abs(reference_t[right] - actual_t)
        nearest = np.where(choose_left, left, right)
        dt = np.diff(actual_t)
        tolerance = max(1e-6, float(np.median(dt[dt > 0])) * 1e-3) if np.any(dt > 0) else 1e-6
        mask = np.abs(reference_t[nearest] - actual_t) <= tolerance
        if np.count_nonzero(mask) < 2:
            return None
        aligned_actual = actual[mask]
        aligned_t = actual_t[mask]
        aligned_reference = reference[nearest[mask]]
        return {
            'times': aligned_t,
            'actual': aligned_actual,
            'reference': aligned_reference,
            'errors': aligned_actual - aligned_reference,
        }

    @staticmethod
    def _wrap_angle(angle: np.ndarray) -> np.ndarray:
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    @staticmethod
    def _draw_planner_segments(
        ax,
        planner_segments: List[np.ndarray],
        is_3d: bool = False,
    ) -> None:
        """批量绘制 PlannerOutput 窗口，保持窗口边界且避免海量 artist。"""
        singleton_points = [
            segment[0] for segment in planner_segments if len(segment) == 1
        ]
        path_segments = [
            segment for segment in planner_segments if len(segment) > 1
        ]
        label = 'Planner output (per message)'

        if path_segments:
            collection_kwargs = {
                'colors': '#7b2cbf',
                'linewidths': 1.8,
                'linestyles': 'dashdot',
                'alpha': .9,
                'zorder': 4,
                'label': label,
                'rasterized': True,
            }
            if is_3d:
                from mpl_toolkits.mplot3d.art3d import Line3DCollection
                ax.add_collection3d(
                    Line3DCollection(path_segments, **collection_kwargs)
                )
            else:
                from matplotlib.collections import LineCollection
                ax.add_collection(
                    LineCollection(
                        [segment[:, :2] for segment in path_segments],
                        **collection_kwargs,
                    )
                )
            label = None

        if singleton_points:
            points = np.asarray(singleton_points, dtype=float)
            scatter_kwargs = {
                'color': '#7b2cbf',
                'marker': '.',
                's': 18,
                'alpha': .85,
                'zorder': 4,
                'label': label,
                'rasterized': True,
            }
            if is_3d:
                ax.scatter(
                    points[:, 0],
                    points[:, 1],
                    points[:, 2],
                    **scatter_kwargs,
                )
            else:
                ax.scatter(points[:, 0], points[:, 1], **scatter_kwargs)

    @staticmethod
    def _draw_trajectory(ax, plot_data: Dict, is_3d: bool = False):
        actual = plot_data['actual']
        reference = plot_data['reference']
        planner_segments = plot_data['planner_segments']
        waypoints = plot_data['waypoints']
        if len(reference):
            if is_3d:
                ax.plot(reference[:, 0], reference[:, 1], reference[:, 2], color='#1677ff', lw=1.6,
                        ls='--', zorder=2, label='Reference (reconstructed)')
            else:
                ax.plot(reference[:, 0], reference[:, 1], color='#1677ff', lw=1.6,
                        ls='--', zorder=2, label='Reference (reconstructed)')
        if len(actual):
            if is_3d:
                ax.plot(actual[:, 0], actual[:, 1], actual[:, 2], color='#f05a28', lw=2.0,
                        zorder=3, label='Actual')
            else:
                ax.plot(actual[:, 0], actual[:, 1], color='#f05a28', lw=2.0, zorder=3, label='Actual')
        BagVisualizer._draw_planner_segments(
            ax,
            planner_segments,
            is_3d=is_3d,
        )
        if len(waypoints):
            if is_3d:
                ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], marker='o', s=26,
                           color='#1b9e77', label='Waypoints')
            else:
                ax.scatter(waypoints[:, 0], waypoints[:, 1], marker='o', s=26,
                           color='#1b9e77', label='Waypoints')
        if len(actual):
            marker_kwargs = {'color': '#2ca02c', 's': 48, 'zorder': 8, 'label': 'Start'}
            end_kwargs = {'color': '#d62728', 's': 62, 'zorder': 9, 'label': 'End'}
            if is_3d:
                ax.scatter(*actual[0], marker='^', **marker_kwargs)
                ax.scatter(*actual[-1], marker='X', **end_kwargs)
            else:
                ax.scatter(actual[0, 0], actual[0, 1], marker='^', **marker_kwargs)
                ax.scatter(actual[-1, 0], actual[-1, 1], marker='X', **end_kwargs)

    def _save_trajectory_plots(self, plt, plot_data: Dict, plot_dir: str, dpi: int) -> Dict:
        actual = plot_data['actual']
        if len(actual) == 0:
            return {}
        all_points = [actual]
        for key in ('reference', 'planner', 'waypoints'):
            if len(plot_data[key]):
                all_points.append(plot_data[key])
        all_points = np.vstack(all_points)
        artifacts = {}

        fig = plt.figure(figsize=(8.2, 6.6))
        ax = fig.add_subplot(111, projection='3d')
        self._draw_trajectory(ax, plot_data, is_3d=True)
        self._set_equal_3d_axes(ax, all_points)
        ax.view_init(elev=27, azim=-58)
        ax.set_title('Flight trajectory (3D)')
        ax.set_xlabel('East x [m]')
        ax.set_ylabel('North y [m]')
        ax.set_zlabel('Up z [m]')
        ax.legend(loc='upper left', bbox_to_anchor=(0.0, 1.0))
        fig.tight_layout()
        path = os.path.join(plot_dir, 'trajectory_3d.png')
        fig.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
        artifacts['trajectory_3d'] = path

        fig, ax = plt.subplots(figsize=(7.2, 6.4))
        self._draw_trajectory(ax, plot_data)
        ax.set_title('Flight trajectory (top view)')
        ax.set_xlabel('East x [m]')
        ax.set_ylabel('North y [m]')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, color='#d9d9d9', lw=.7)
        ax.legend(loc='best')
        fig.tight_layout()
        path = os.path.join(plot_dir, 'trajectory_xy.png')
        fig.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
        artifacts['trajectory_xy'] = path
        return artifacts

    def _save_state_plots(
        self,
        plt,
        plot_data: Dict,
        plot_dir: str,
        dpi: int,
        report: Optional[AnalysisReport] = None,
    ) -> Dict:
        actual, actual_t = plot_data['actual'], plot_data['actual_t']
        reference, reference_t = plot_data['reference'], plot_data['reference_t']
        artifacts = {}
        if len(actual):
            elapsed = actual_t - plot_data['start_time']
            fig, axes = plt.subplots(3, 1, figsize=(8.4, 7.0), sharex=True)
            for axis, label, index in zip(axes, ('x', 'y', 'z'), range(3)):
                axis.plot(elapsed, actual[:, index], color='#f05a28', lw=1.5, label='Actual')
                if len(reference):
                    axis.plot(reference_t - plot_data['start_time'], reference[:, index], color='#1677ff',
                              lw=1.3, ls='--', label='Reference')
                axis.set_ylabel(f'{label} [m]')
                axis.grid(True, color='#e1e1e1', lw=.7)
            axes[0].legend(loc='best')
            axes[0].set_title('Position tracking')
            axes[-1].set_xlabel('Time from selected range [s]')
            fig.tight_layout()
            path = os.path.join(plot_dir, 'position_tracking.png')
            fig.savefig(path, dpi=dpi, bbox_inches='tight')
            plt.close(fig)
            artifacts['position_tracking'] = path

            alignment = self._tracking_alignment(plot_data)
            if alignment is not None:
                error_t = alignment['times'] - plot_data['start_time']
                errors = alignment['errors']
                error_norm = np.linalg.norm(errors, axis=1)
                fig, axes = plt.subplots(4, 1, figsize=(8.4, 8.0), sharex=True)
                for index, (axis, label, color) in enumerate(zip(
                    axes[:3], ('x error', 'y error', 'z error'), ('#e76f51', '#2a9d8f', '#457b9d'),
                )):
                    axis.plot(error_t, errors[:, index], color=color, lw=1.25)
                    axis.axhline(0.0, color='#555555', lw=.7)
                    axis.set_ylabel(f'{label} [m]')
                    axis.grid(True, color='#e1e1e1', lw=.7)
                axes[3].plot(error_t, error_norm, color='#c62828', lw=1.5, label='3D error')
                axes[3].fill_between(error_t, 0, error_norm, color='#c62828', alpha=.14)
                axes[3].set_ylabel('3D error [m]')
                axes[3].set_xlabel('Time from selected range [s]')
                axes[3].grid(True, color='#e1e1e1', lw=.7)
                tracking_metric = self._find_metric(report, 'position_tracking_rmse_3d')
                p95_metric = self._find_metric(report, 'position_tracking_p95_3d')
                labels = []
                if tracking_metric and not math.isnan(tracking_metric.value):
                    labels.append(f'3D RMSE = {tracking_metric.value:.3f} m')
                if p95_metric and not math.isnan(p95_metric.value):
                    labels.append(f'3D P95 = {p95_metric.value:.3f} m')
                axes[0].set_title('Position tracking error' + (f"  |  {', '.join(labels)}" if labels else ''))
                fig.tight_layout()
                path = os.path.join(plot_dir, 'position_tracking_error.png')
                fig.savefig(path, dpi=dpi, bbox_inches='tight')
                plt.close(fig)
                artifacts['position_tracking_error'] = path

        attitude, attitude_t = plot_data['attitude'], plot_data['attitude_t']
        thrust, thrust_t = plot_data['thrust'], plot_data['thrust_t']
        if len(attitude) or len(thrust):
            fig, axes = plt.subplots(2, 1, figsize=(8.4, 5.5), sharex=False)
            if len(attitude):
                for idx, (name, color) in enumerate((('Roll', '#e76f51'), ('Pitch', '#2a9d8f'), ('Yaw', '#457b9d'))):
                    axes[0].plot(attitude_t - plot_data['start_time'], np.rad2deg(attitude[:, idx]),
                                 color=color, lw=1.1, label=name)
                axes[0].set_ylabel('Angle [deg]')
                attitude_metric = (
                    self._find_metric(report, 'roll_pitch_fluctuation_rms')
                    or self._find_metric(report, 'attitude_fluctuation_rms')
                )
                attitude_suffix = '' if attitude_metric is None or math.isnan(attitude_metric.value) else (
                    f'  |  RMS = {math.degrees(attitude_metric.value):.2f}°'
                )
                axes[0].set_title('Attitude response' + attitude_suffix)
                axes[0].legend(loc='best', ncol=3)
                axes[0].grid(True, color='#e1e1e1', lw=.7)
            else:
                axes[0].set_visible(False)
            if len(thrust):
                axes[1].plot(thrust_t - plot_data['start_time'], thrust, color='#6a4c93', lw=1.2)
                axes[1].set_ylabel('Normalized thrust')
                axes[1].set_xlabel('Time from selected range [s]')
                thrust_metric = self._find_metric(report, 'thrust_std')
                thrust_suffix = '' if thrust_metric is None or math.isnan(thrust_metric.value) else (
                    f'  |  std = {thrust_metric.value:.4f}'
                )
                axes[1].set_title('Thrust command' + thrust_suffix)
                axes[1].grid(True, color='#e1e1e1', lw=.7)
            else:
                axes[1].set_visible(False)
            fig.tight_layout()
            path = os.path.join(plot_dir, 'attitude_thrust.png')
            fig.savefig(path, dpi=dpi, bbox_inches='tight')
            plt.close(fig)
            artifacts['attitude_thrust'] = path
        return artifacts

    def _save_yaw_tracking_plot(self, plt, plot_data: Dict, plot_dir: str, dpi: int,
                                report: Optional[AnalysisReport] = None) -> Dict:
        """绘制实际/参考 yaw 与包裹后的 yaw 跟踪误差。"""
        attitude, attitude_t = plot_data['attitude'], plot_data['attitude_t']
        reference_yaws, reference_yaw_t = plot_data['reference_yaws'], plot_data['reference_yaw_t']
        valid_ref_yaw = np.isfinite(reference_yaws)
        if len(attitude) < 2 or np.count_nonzero(valid_ref_yaw) < 2:
            return {}
        valid_times = reference_yaw_t[valid_ref_yaw]
        valid_yaws = reference_yaws[valid_ref_yaw]
        indices = np.searchsorted(valid_times, attitude_t)
        right = np.clip(indices, 0, len(valid_times) - 1)
        left = np.clip(indices - 1, 0, len(valid_times) - 1)
        choose_left = np.abs(valid_times[left] - attitude_t) <= np.abs(valid_times[right] - attitude_t)
        nearest = np.where(choose_left, left, right)
        dt = np.diff(attitude_t)
        tolerance = max(1e-6, float(np.median(dt[dt > 0])) * 1e-3) if np.any(dt > 0) else 1e-6
        mask = np.abs(valid_times[nearest] - attitude_t) <= tolerance
        if np.count_nonzero(mask) < 2:
            return {}
        times = attitude_t[mask]
        actual_yaw = attitude[mask, 2]
        ref_yaw = np.unwrap(valid_yaws[nearest[mask]])
        yaw_error = self._wrap_angle(actual_yaw - ref_yaw)
        elapsed = times - plot_data['start_time']
        metric = self._find_metric(report, 'yaw_tracking_rmse')
        suffix = '' if metric is None or not metric.available else f'  |  RMSE = {math.degrees(metric.value):.2f}°'

        fig, axes = plt.subplots(2, 1, figsize=(8.4, 5.6), sharex=True)
        axes[0].plot(elapsed, np.rad2deg(np.unwrap(actual_yaw)), color='#457b9d', lw=1.25, label='Actual yaw')
        axes[0].plot(elapsed, np.rad2deg(ref_yaw), color='#1677ff', lw=1.2, ls='--', label='Reference yaw')
        axes[0].set_title('Yaw tracking' + suffix)
        axes[0].set_ylabel('Yaw [deg]')
        axes[0].legend(loc='best')
        axes[0].grid(True, color='#e1e1e1', lw=.7)
        axes[1].plot(elapsed, np.rad2deg(yaw_error), color='#c62828', lw=1.25)
        axes[1].axhline(0.0, color='#555555', lw=.7)
        axes[1].set_xlabel('Time from selected range [s]')
        axes[1].set_ylabel('Yaw error [deg]')
        axes[1].grid(True, color='#e1e1e1', lw=.7)
        fig.tight_layout()
        path = os.path.join(plot_dir, 'yaw_tracking_error.png')
        fig.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
        return {'yaw_tracking_error': path}

    def _save_hover_stability_plot(self, plt, plot_data: Dict, plot_dir: str, dpi: int,
                                   report: Optional[AnalysisReport] = None) -> Dict:
        """单独诊断悬停抖动和 Z 轴稳态误差。"""
        if self.analyzer.task.name != 'hover' or len(plot_data['actual']) < 2:
            return {}
        target_z = report.hover_height if report is not None else self.analyzer.task.get_hover_height()
        target = np.array([0.0, 0.0, target_z])
        deviations = plot_data['actual'] - target
        elapsed = plot_data['actual_t'] - plot_data['start_time']
        jitter = np.linalg.norm(deviations, axis=1)
        jitter_metric = self._find_metric(report, 'position_jitter_rms')
        z_metric = self._find_metric(report, 'z_steady_state_error')
        title_bits = []
        if jitter_metric and not math.isnan(jitter_metric.value):
            title_bits.append(f'jitter RMS = {jitter_metric.value:.3f} m')
        if z_metric and not math.isnan(z_metric.value):
            title_bits.append(f'Z bias = {z_metric.value:.3f} m')

        fig, axes = plt.subplots(2, 1, figsize=(8.4, 5.6), sharex=True)
        for index, (label, color) in enumerate((('x', '#e76f51'), ('y', '#2a9d8f'), ('z', '#457b9d'))):
            axes[0].plot(elapsed, deviations[:, index], color=color, lw=1.1, label=f'{label} deviation')
        axes[0].axhline(0.0, color='#555555', lw=.7)
        axes[0].set_title('Hover stability' + (f"  |  {', '.join(title_bits)}" if title_bits else ''))
        axes[0].set_ylabel('Position deviation [m]')
        axes[0].legend(loc='best', ncol=3)
        axes[0].grid(True, color='#e1e1e1', lw=.7)
        axes[1].plot(elapsed, jitter, color='#6a4c93', lw=1.3, label='3D deviation')
        axes[1].fill_between(elapsed, 0, jitter, color='#6a4c93', alpha=.12)
        axes[1].set_xlabel('Time from selected range [s]')
        axes[1].set_ylabel('3D deviation [m]')
        axes[1].grid(True, color='#e1e1e1', lw=.7)
        fig.tight_layout()
        path = os.path.join(plot_dir, 'hover_stability.png')
        fig.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
        return {'hover_stability': path}

    @staticmethod
    def _amplitude_spectrum(values: np.ndarray, times: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """复用振荡指标的预处理，返回 0.5--20 Hz 内的单边幅值谱。"""
        values = np.asarray(values, dtype=float)
        times = np.asarray(times, dtype=float)
        valid = np.isfinite(values) & np.isfinite(times)
        values, times = values[valid], times[valid]
        if len(values) < 64:
            return None
        order = np.argsort(times)
        values, times = values[order], times[order]
        times, unique_indices = np.unique(times, return_index=True)
        values = values[unique_indices]
        if len(values) < 64 or times[-1] <= times[0]:
            return None
        dt = float(np.median(np.diff(times)))
        if dt <= 0:
            return None
        uniform_times = np.arange(times[0], times[-1] + .5 * dt, dt)
        values = np.interp(uniform_times, times, values)
        x = np.arange(len(values), dtype=float)
        detrended = values - np.polyval(np.polyfit(x, values, 1), x)
        window = np.hanning(len(values))
        amplitudes = 2.0 * np.abs(np.fft.rfft(detrended * window)) / max(float(np.sum(window)), 1.0)
        frequencies = np.fft.rfftfreq(len(values), d=dt)
        mask = (frequencies >= .5) & (frequencies <= 20.0)
        return (frequencies[mask], amplitudes[mask]) if np.any(mask) else None

    def _save_oscillation_spectrum(self, plt, plot_data: Dict, plot_dir: str, dpi: int) -> Dict:
        """单独生成与 detect_oscillation 对应的频谱诊断图。"""
        signals: List[Tuple[str, np.ndarray, np.ndarray, str]] = []
        if self.analyzer.task.name == 'hover':
            actual = plot_data['actual']
            actual_t = plot_data['actual_t']
            if len(actual):
                signals.extend([
                    ('Position X', actual[:, 0], actual_t, 'm'),
                    ('Position Z', actual[:, 2], actual_t, 'm'),
                ])
        else:
            alignment = self._tracking_alignment(plot_data)
            if alignment is not None:
                signals.append(('3D tracking error', np.linalg.norm(alignment['errors'], axis=1),
                                alignment['times'], 'm'))
        attitude, attitude_t = plot_data['attitude'], plot_data['attitude_t']
        if len(attitude):
            signals.append(('Roll', np.rad2deg(attitude[:, 0]), attitude_t, 'deg'))
        thrust, thrust_t = plot_data['thrust'], plot_data['thrust_t']
        if len(thrust):
            signals.append(('Thrust', thrust, thrust_t, 'normalized'))

        spectra = []
        for label, values, times, unit in signals:
            spectrum = self._amplitude_spectrum(values, times)
            if spectrum is not None:
                spectra.append((label, spectrum[0], spectrum[1], unit))
        if not spectra:
            return {}
        fig, axes = plt.subplots(len(spectra), 1, figsize=(8.4, max(3.0, 2.25 * len(spectra))), sharex=True)
        axes = np.atleast_1d(axes)
        for axis, (label, frequencies, amplitudes, unit) in zip(axes, spectra):
            axis.plot(frequencies, amplitudes, color='#6a4c93', lw=1.15)
            axis.set_ylabel(f'Amplitude [{unit}]')
            axis.set_title(label, loc='left', fontsize=10)
            axis.grid(True, color='#e1e1e1', lw=.7)
        axes[-1].set_xlabel('Frequency [Hz]')
        fig.suptitle('Oscillation diagnostics (detrended Hann-window spectrum)', y=1.01)
        fig.tight_layout()
        path = os.path.join(plot_dir, 'oscillation_spectrum.png')
        fig.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
        return {'oscillation_spectrum': path}

    def _save_core_plots(self, plt, plot_data: Dict, plot_dir: str, dpi: int,
                         report: Optional[AnalysisReport] = None) -> Dict:
        """每次 run/analyze 自动生成任务语义对应的核心图。"""
        artifacts = {}
        if self.analyzer.task.render_trajectory_plots:
            artifacts.update(self._save_trajectory_plots(plt, plot_data, plot_dir, dpi))
        artifacts.update(self._save_state_plots(plt, plot_data, plot_dir, dpi, report))
        artifacts.update(self._save_yaw_tracking_plot(plt, plot_data, plot_dir, dpi, report))
        artifacts.update(self._save_metrics_overview(plt, report, plot_dir, dpi))
        return artifacts

    def _save_metrics_overview(self, plt, report: Optional[AnalysisReport], plot_dir: str, dpi: int) -> Dict:
        if report is None:
            return {}
        metrics = [metric for metric in report.metrics if metric.available]
        if not metrics:
            return {}
        group_colors = {
            'outcome': '#4e79a7', 'tracking': '#59a14f',
            'trajectory_quality': '#f28e2b', 'navigation': '#76b7b2',
            'stability': '#af7aa1', 'interface_health': '#9c755f',
            'diagnostic': '#79706e',
        }
        fig_height = max(3.3, .42 * len(metrics) + 1.3)
        fig, ax = plt.subplots(figsize=(10.0, fig_height))
        y_values = np.arange(len(metrics))[::-1]
        colors = [group_colors.get(metric.group, '#79706e') for metric in metrics]
        ax.scatter(np.ones(len(metrics)), y_values, s=115, c=colors, marker='s')
        for y, metric in zip(y_values, metrics):
            ax.text(.92, y, self._metric_label(metric), va='center', ha='right', fontsize=9)
            ax.text(1.08, y, self._format_metric(metric), va='center', ha='left', fontsize=9)
        ax.set_xlim(.35, 2.05)
        ax.set_ylim(-.8, len(metrics) - .2)
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.set_title('Evaluation metrics overview')
        legend_handles = [
            plt.Line2D([], [], color=color, marker='s', linestyle='', label=group)
            for group, color in group_colors.items()
            if any(metric.group == group for metric in metrics)
        ]
        if legend_handles:
            ax.legend(handles=legend_handles, loc='lower center', ncol=min(5, len(legend_handles)),
                      bbox_to_anchor=(.5, -0.14), frameon=False)
        fig.tight_layout()
        path = os.path.join(plot_dir, 'metrics_overview.png')
        fig.savefig(path, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
        return {'metrics_overview': path}

    def _save_replay_gif(
        self, plt, PillowWriter, plot_data: Dict, plot_dir: str, fps: int,
        playback_seconds: float, dpi: int,
    ) -> Dict:
        actual, actual_t = plot_data['actual'], plot_data['actual_t']
        if len(actual) < 2:
            return {'warning': '实际位姿数据不足，未生成飞行回放 GIF'}
        if fps <= 0 or playback_seconds <= 0:
            raise ValueError('--gif-fps 和 --gif-duration 必须为正数。')

        frame_count = max(2, int(round(fps * playback_seconds)))
        frame_times = np.linspace(actual_t[0], actual_t[-1], frame_count)
        frame_positions = np.column_stack([
            np.interp(frame_times, actual_t, actual[:, index]) for index in range(3)
        ])
        attitude, attitude_t = plot_data['attitude'], plot_data['attitude_t']
        if len(attitude) >= 2:
            frame_attitudes = np.column_stack([
                np.interp(frame_times, attitude_t, np.unwrap(attitude[:, index])) for index in range(3)
            ])
        else:
            frame_attitudes = np.zeros((frame_count, 3))

        all_points = [actual]
        for key in ('reference', 'planner', 'waypoints'):
            if len(plot_data[key]):
                all_points.append(plot_data[key])
        all_points = np.vstack(all_points)
        arm_length = min(max(float(np.max(np.ptp(all_points, axis=0))) * .055, .16), .55)

        fig = plt.figure(figsize=(7.6, 6.3))
        ax = fig.add_subplot(111, projection='3d')
        self._draw_trajectory(ax, plot_data, is_3d=True)
        self._set_equal_3d_axes(ax, all_points)
        ax.view_init(elev=27, azim=-58)
        ax.set_xlabel('East x [m]')
        ax.set_ylabel('North y [m]')
        ax.set_zlabel('Up z [m]')
        trace, = ax.plot([], [], [], color='#f05a28', lw=2.7, label='Replay trace')
        arm_a, = ax.plot([], [], [], color='#202124', lw=2.6)
        arm_b, = ax.plot([], [], [], color='#202124', lw=2.6)
        body, = ax.plot([], [], [], color='#d81b60', marker='o', markersize=5)
        ax.legend(loc='upper left', bbox_to_anchor=(0.0, 1.0))

        def update(frame_index):
            position = frame_positions[frame_index]
            rotation = self._rotation_matrix(*frame_attitudes[frame_index])
            arms = rotation @ np.array([
                [-arm_length, arm_length, 0, 0], [0, 0, -arm_length, arm_length], [0, 0, 0, 0],
            ])
            arm_a.set_data_3d(position[0] + arms[0, :2], position[1] + arms[1, :2], position[2] + arms[2, :2])
            arm_b.set_data_3d(position[0] + arms[0, 2:], position[1] + arms[1, 2:], position[2] + arms[2, 2:])
            body.set_data_3d([position[0]], [position[1]], [position[2]])
            trace.set_data_3d(frame_positions[:frame_index + 1, 0], frame_positions[:frame_index + 1, 1],
                              frame_positions[:frame_index + 1, 2])
            elapsed = frame_times[frame_index] - plot_data['start_time']
            ax.set_title(f'Flight replay  |  t = {elapsed:.2f} s')
            return trace, arm_a, arm_b, body

        path = os.path.join(plot_dir, 'flight_replay.gif')
        try:
            from matplotlib.animation import FuncAnimation
            animation = FuncAnimation(fig, update, frames=frame_count, interval=1000 / fps, blit=False)
            animation.save(path, writer=PillowWriter(fps=fps), dpi=dpi)
        finally:
            plt.close(fig)
        return {'flight_replay_gif': path}

    def render_trajectory_plots(
        self,
        output_dir: str,
        full_data: Optional[Dict] = None,
        phase_data: Optional[Dict] = None,
        scope: str = 'execution',
        dpi: int = 220,
    ) -> Dict:
        """只生成 3D/XY 轨迹图；可复用分析阶段已缓存的 bag 数据。"""
        if scope not in ('execution', 'full'):
            raise ValueError("scope 必须为 'execution' 或 'full'。")
        if dpi <= 0:
            raise ValueError('--dpi 必须为正整数。')
        if full_data is None or phase_data is None:
            _, full_data, time_range = self._collect_data(scope)
        else:
            time_range = self._resolve_time_range(full_data, phase_data, scope)
        plot_data = self._build_plot_data(full_data, time_range)
        if len(plot_data['actual']) == 0:
            raise ValueError('所选时间范围没有实际位姿数据，无法生成轨迹图。')
        plt, _ = self._import_plotting()
        plot_dir = os.path.join(os.path.abspath(output_dir), PLOTS_DIRNAME)
        os.makedirs(plot_dir, exist_ok=True)
        return self._save_trajectory_plots(plt, plot_data, plot_dir, dpi)

    def render_core_plots(
        self,
        output_dir: str,
        report: Optional[AnalysisReport] = None,
        full_data: Optional[Dict] = None,
        phase_data: Optional[Dict] = None,
        scope: str = 'execution',
        dpi: int = 220,
    ) -> Dict:
        """生成每个 task 的轨迹、位置误差、姿态/推力和可用 yaw 误差图。"""
        if scope not in ('execution', 'full'):
            raise ValueError("scope 必须为 'execution' 或 'full'。")
        if dpi <= 0:
            raise ValueError('--dpi 必须为正整数。')
        if full_data is None or phase_data is None:
            collected_report, full_data, time_range = self._collect_data(scope)
            report = report or collected_report
        else:
            time_range = self._resolve_time_range(full_data, phase_data, scope)
        plot_data = self._build_plot_data(full_data, time_range)
        if len(plot_data['actual']) == 0:
            raise ValueError('所选时间范围没有实际位姿数据，无法生成核心图。')
        plt, _ = self._import_plotting()
        plot_dir = os.path.join(os.path.abspath(output_dir), PLOTS_DIRNAME)
        os.makedirs(plot_dir, exist_ok=True)
        return self._save_core_plots(plt, plot_data, plot_dir, dpi, report)

    def render(
        self,
        output_dir: str,
        scope: str = 'execution',
        gif: bool = False,
        gif_fps: int = 15,
        gif_duration: float = 12.0,
        dpi: int = 220,
        diagnostics: Optional[List[str]] = None,
    ) -> Dict:
        """生成可视化产物并返回 ``{name: absolute_path}`` 映射。"""
        if scope not in ('execution', 'full'):
            raise ValueError("scope 必须为 'execution' 或 'full'。")
        if dpi <= 0:
            raise ValueError('--dpi 必须为正整数。')
        plt, PillowWriter = self._import_plotting()
        report, data, time_range = self._collect_data(scope)
        plot_data = self._build_plot_data(data, time_range)
        if len(plot_data['actual']) == 0:
            raise ValueError('所选时间范围没有实际位姿数据，无法生成可视化。')

        plot_dir = os.path.join(os.path.abspath(output_dir), PLOTS_DIRNAME)
        os.makedirs(plot_dir, exist_ok=True)
        diagnostics = set(diagnostics or [])
        if 'all' in diagnostics:
            diagnostics.update(('hover_stability', 'oscillation'))
        artifacts = self._save_core_plots(plt, plot_data, plot_dir, dpi, report)
        if 'hover_stability' in diagnostics:
            artifacts.update(self._save_hover_stability_plot(plt, plot_data, plot_dir, dpi, report))
        if 'oscillation' in diagnostics:
            artifacts.update(self._save_oscillation_spectrum(plt, plot_data, plot_dir, dpi))
        if report is None:
            artifacts['warning'] = 'bag 缺少 /flight_state：已生成完整飞行图和回放，未生成指标概览。'
        if gif:
            artifacts.update(self._save_replay_gif(
                plt, PillowWriter, plot_data, plot_dir, gif_fps, gif_duration, dpi,
            ))
        return artifacts
