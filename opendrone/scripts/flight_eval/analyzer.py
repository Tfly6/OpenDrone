#!/usr/bin/env python3
"""Extract task phases and metrics from a rosbag."""

import json
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

from .metrics import (
    AnalysisReport,
    MetricResult,
    MetricsCalculator,
)
from .controllers import CONTROLLER_REGISTRY
from .outcomes import (
    MISSION_PATH_TOPIC,
    TRAJECTORY_TRIGGER_TOPIC,
    TaskOutcome,
)
from .planners import get_planner_launch
from .tasks import create_task


FLIGHT_STATE_MAP = {
    0: 'WAITING_FOR_CONNECTED',
    1: 'WAITING_FOR_OFFBOARD',
    2: 'TAKEOFF',
    3: 'MISSION_EXECUTION',
    4: 'LANDING',
    5: 'LANDED',
    6: 'EMERGENCY',
}

GROUP_LABELS = {
    'outcome': '任务结果',
    'tracking': '跟踪',
    'execution_tracking': '执行跟踪',
    'spatial_fidelity': '空间形状',
    'trajectory_quality': '轨迹质量',
    'navigation': '导航',
    'stability': '稳定性',
    'interface_health': '接口健康',
    'diagnostic': '诊断',
}

REPORT_FILENAME = 'report.json'
AGENT_SUMMARY_FILENAME = 'agent_summary.md'
PARAMETER_SNAPSHOT_MANIFEST_FILENAME = 'parameter_snapshot.json'
ROSPARAM_SNAPSHOT_FILENAME = 'rosparams.json'


class NumpyEncoder(json.JSONEncoder):
    """JSON 编码器: 自动转换 numpy 类型为 Python 原生类型"""
    def default(self, obj):
        if isinstance(obj, np.bool_):
            return bool(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return None if math.isnan(obj) else float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)


class BagAnalyzer:
    """Bag 文件分析器"""

    def __init__(self, bag_file: str, controller_name: str,
                 task_name: str = 'hover',
                 planner_name: str = 'none',
                 hover_height: Optional[float] = None,
                 duration: Optional[float] = None,
                 recompute_outcome: bool = False):
        self.bag_file = bag_file
        self.controller_name = controller_name
        if controller_name not in CONTROLLER_REGISTRY:
            raise ValueError(f'未知控制器: {controller_name}')
        self.task = create_task(
            task_name,
            takeoff_height=hover_height or 2.0,
            duration=duration,
        )
        self.planner_name = (
            self.task.planner_name_override
            if planner_name == 'none' and self.task.planner_name_override
            else planner_name
        )
        self.controller_topics = CONTROLLER_REGISTRY[controller_name]['evaluation_topics']
        self.planner_info = get_planner_launch(self.planner_name)
        self.planner_topics = self.planner_info['evaluation_topics']
        self.hover_height = hover_height
        self.calculator = MetricsCalculator(hover_height=hover_height or 2.0)
        self.run_metadata = self._load_run_metadata()
        # The recorded Runner result remains the default because it documents
        # what happened at runtime.  Offline evaluator changes must opt in.
        self.recompute_outcome = recompute_outcome
        self._last_full_data = None
        self._last_phase_data = None

    def _load_run_metadata(self) -> Dict:
        path = os.path.join(
            os.path.dirname(os.path.abspath(self.bag_file)),
            'run_metadata.json',
        )
        if not os.path.isfile(path):
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as metadata_file:
                value = json.load(metadata_file)
            return value if isinstance(value, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def _extract_bag_data(self) -> Dict:
        """Extract recorded state, reference, and task streams."""
        try:
            import rosbag
        except ImportError:
            raise ImportError(
                "需要 rosbag 库。请运行: sudo apt install python3-rosbag 或 pip install rosbag"
            )

        positions = []
        position_times = []
        position_frames = []
        velocities = []
        velocity_times = []
        attitudes = []
        attitude_times = []
        thrusts = []
        thrust_times = []
        planner_reference_windows = []
        planner_mission_states = []
        planner_output_positions = []
        planner_output_message_times = []
        planner_output_times = []
        planner_output_point_message_times = []
        waypoint_positions = []
        waypoint_times = []
        mission_paths = []
        trigger_times = []
        flight_states = []
        flight_state_times = []

        bag = rosbag.Bag(self.bag_file)

        recorded_topics = [
            '/mavros/local_position/odom',
            '/mavros/setpoint_raw/attitude',
        ] + list(self.controller_topics.values()) + list(self.planner_topics.values())
        recorded_topics += self.task.record_topics
        recorded_topics = list(dict.fromkeys(recorded_topics))

        for topic, msg, t in bag.read_messages(topics=recorded_topics):
            stamp = t.to_sec()

            if topic == '/mavros/local_position/odom':
                p = msg.pose.pose.position
                positions.append([p.x, p.y, p.z])
                position_times.append(stamp)
                position_frames.append(str(getattr(msg.header, 'frame_id', '') or ''))
                q = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                velocities.append(self._rotate_vector_by_quaternion(
                    q.w, q.x, q.y, q.z, np.array([v.x, v.y, v.z], dtype=float),
                ))
                velocity_times.append(stamp)
                roll, pitch, yaw = self._quat_to_euler(q.w, q.x, q.y, q.z)
                attitudes.append([roll, pitch, yaw])
                attitude_times.append(stamp)

            elif topic == '/mavros/setpoint_raw/attitude':
                thrusts.append(msg.thrust)
                thrust_times.append(stamp)

            elif topic == self.planner_topics.get('planner_output'):
                planner_output_message_times.append(stamp)
                traj_start_time = msg.trajectory_start_time.to_sec()
                trajectory_id = int(msg.trajectory_id)
                window_points = []
                for point in getattr(msg, 'points', []):
                    if not (point.valid_mask & 1):
                        continue
                    point_time = traj_start_time + point.time_from_start.to_sec()
                    valid_mask = int(point.valid_mask)
                    def vector_or_nan(vector, mask):
                        if valid_mask & mask:
                            return np.array([vector.x, vector.y, vector.z], dtype=float)
                        return np.full(3, np.nan, dtype=float)
                    point_data = {
                        'time': point_time,
                        'time_from_start': float(point.time_from_start.to_sec()),
                        'trajectory_start_time': traj_start_time,
                        'trajectory_id': trajectory_id,
                        'valid_mask': valid_mask,
                        'position': np.array([
                            point.position.x, point.position.y, point.position.z,
                        ], dtype=float),
                        'velocity': vector_or_nan(point.velocity, 2),
                        'acceleration': vector_or_nan(point.acceleration, 4),
                        'jerk': vector_or_nan(point.jerk, 8),
                        'snap': vector_or_nan(point.snap, 16),
                        'yaw': float(point.yaw) if valid_mask & 32 else float('nan'),
                        'yaw_rate': float(point.yaw_rate) if valid_mask & 64 else float('nan'),
                        'angular_velocity': vector_or_nan(point.angular_velocity, 128),
                    }
                    window_points.append(point_data)
                    planner_output_positions.append([
                        point.position.x,
                        point.position.y,
                        point.position.z,
                    ])
                    planner_output_times.append(point_time)
                    planner_output_point_message_times.append(stamp)
                if window_points:
                    window_points.sort(key=lambda item: item['time'])
                    planner_reference_windows.append({
                        'message_time': stamp,
                        'frame_id': str(getattr(msg.header, 'frame_id', '') or ''),
                        'trajectory_id': trajectory_id,
                        'trajectory_start_time': traj_start_time,
                        'is_horizon': bool(getattr(msg, 'is_horizon', False)),
                        'is_single': len(window_points) == 1,
                        'trajectory_status': int(
                            getattr(msg, 'trajectory_status', 1) or 0
                        ),
                        'points': window_points,
                    })

            elif topic == self.planner_topics.get('mission_state'):
                planner_mission_states.append({
                    'message_time': stamp,
                    'mission_type': int(msg.mission_type),
                    'status': int(msg.status),
                    'completed_items': int(msg.completed_items),
                    'total_items': int(msg.total_items),
                    'detail': str(msg.detail or ''),
                })

            elif topic == MISSION_PATH_TOPIC:
                path_positions = []
                for pose_stamped in getattr(msg, 'poses', []):
                    p = pose_stamped.pose.position
                    path_positions.append([p.x, p.y, p.z])
                    waypoint_positions.append([p.x, p.y, p.z])
                    point_stamp = pose_stamped.header.stamp.to_sec()
                    waypoint_times.append(point_stamp if point_stamp > 0.0 else stamp)
                mission_paths.append({
                    'time': stamp,
                    'frame_id': str(getattr(msg.header, 'frame_id', '') or ''),
                    'positions': path_positions,
                })

            elif topic == TRAJECTORY_TRIGGER_TOPIC:
                trigger_times.append(stamp)

            elif topic == self.controller_topics['flight_state']:
                flight_states.append(msg.data)
                flight_state_times.append(stamp)

        bag.close()

        query_times = np.unique(np.concatenate([
            np.asarray(position_times, dtype=float),
            np.asarray(attitude_times, dtype=float),
        ])) if position_times or attitude_times else np.empty(0)
        reference = self._reconstruct_planner_reference(planner_reference_windows, query_times)

        return {
            'positions': np.array(positions) if positions else np.empty((0, 3)),
            'position_times': np.array(position_times) if position_times else np.empty(0),
            'position_frames': np.array(position_frames, dtype=object) if position_frames else np.empty(0, dtype=object),
            'velocities': np.array(velocities) if velocities else np.empty((0, 3)),
            'velocity_times': np.array(velocity_times) if velocity_times else np.empty(0),
            'attitudes': np.array(attitudes) if attitudes else np.empty((0, 3)),
            'attitude_times': np.array(attitude_times) if attitude_times else np.empty(0),
            'thrusts': np.array(thrusts) if thrusts else np.empty(0),
            'thrust_times': np.array(thrust_times) if thrust_times else np.empty(0),
            'reference_positions': reference['positions'],
            'ref_pos_times': reference['times'],
            'reference_velocities': reference['velocities'],
            'reference_accelerations': reference['accelerations'],
            'reference_jerks': reference['jerks'],
            'reference_yaws': reference['yaws'],
            'reference_valid_masks': reference['valid_masks'],
            'planner_output_windows': planner_reference_windows,
            'planner_mission_states': planner_mission_states,
            'planner_trajectories': self._deduplicate_planner_trajectories(planner_reference_windows),
            'planner_output_positions': np.array(planner_output_positions) if planner_output_positions else np.empty((0, 3)),
            'planner_output_times': np.array(planner_output_times) if planner_output_times else np.empty(0),
            'planner_output_message_times': np.array(planner_output_message_times) if planner_output_message_times else np.empty(0),
            'planner_output_point_message_times': np.array(planner_output_point_message_times) if planner_output_point_message_times else np.empty(0),
            'waypoint_positions': np.array(waypoint_positions) if waypoint_positions else np.empty((0, 3)),
            'waypoint_times': np.array(waypoint_times) if waypoint_times else np.empty(0),
            'mission_paths': mission_paths,
            'trigger_times': np.array(trigger_times) if trigger_times else np.empty(0),
            'flight_states': np.array(flight_states) if flight_states else np.empty(0),
            'flight_state_times': np.array(flight_state_times) if flight_state_times else np.empty(0),
        }

    @staticmethod
    def _interpolate_planner_points(points: List[Dict], query_time: float) -> Optional[Dict]:
        """Linearly sample a time-ordered PlannerOutput point sequence."""
        if not points or query_time < points[0]['time'] or query_time > points[-1]['time']:
            return None
        if len(points) == 1:
            return points[0]
        for point in points:
            if abs(float(point['time']) - query_time) <= 1e-9:
                return point

        for right_index in range(1, len(points)):
            right = points[right_index]
            if query_time > right['time']:
                continue
            left = points[right_index - 1]
            interval = right['time'] - left['time']
            if interval <= 0.0:
                return right
            ratio = (query_time - left['time']) / interval
            yaw = float('nan')
            if np.isfinite(left['yaw']) and np.isfinite(right['yaw']):
                delta = (right['yaw'] - left['yaw'] + math.pi) % (2.0 * math.pi) - math.pi
                yaw = left['yaw'] + ratio * delta
            interpolated = {
                'position': left['position'] + ratio * (right['position'] - left['position']),
                'yaw': yaw,
                'valid_mask': int(left['valid_mask'] & right['valid_mask']),
                'trajectory_id': right['trajectory_id'],
            }
            for key in ('velocity', 'acceleration', 'jerk'):
                left_value = left[key]
                right_value = right[key]
                interpolated[key] = (
                    left_value + ratio * (right_value - left_value)
                    if np.all(np.isfinite(left_value)) and np.all(np.isfinite(right_value))
                    else np.full(3, np.nan, dtype=float)
                )
            return interpolated
        return points[-1]

    @classmethod
    def _reconstruct_planner_reference(cls, windows: List[Dict], query_times: np.ndarray) -> Dict:
        """Sample the active PlannerOutput at state timestamps."""
        result = {
            'times': [], 'positions': [], 'velocities': [],
            'accelerations': [], 'jerks': [], 'yaws': [], 'valid_masks': [],
        }
        for query_time in query_times:
            active = None
            for window in reversed(windows):
                if window['message_time'] > query_time:
                    continue
                points = window['points']
                if window['is_single']:
                    active = points[0]
                else:
                    active = cls._interpolate_planner_points(points, query_time)
                break
            if active is None:
                continue
            result['times'].append(query_time)
            result['positions'].append(active['position'])
            result['velocities'].append(active['velocity'])
            result['accelerations'].append(active['acceleration'])
            result['jerks'].append(active['jerk'])
            result['yaws'].append(active['yaw'])
            result['valid_masks'].append(active['valid_mask'])

        return {
            'times': np.asarray(result['times'], dtype=float) if result['times'] else np.empty(0),
            'positions': np.asarray(result['positions'], dtype=float) if result['positions'] else np.empty((0, 3)),
            'velocities': np.asarray(result['velocities'], dtype=float) if result['velocities'] else np.empty((0, 3)),
            'accelerations': np.asarray(result['accelerations'], dtype=float) if result['accelerations'] else np.empty((0, 3)),
            'jerks': np.asarray(result['jerks'], dtype=float) if result['jerks'] else np.empty((0, 3)),
            'yaws': np.asarray(result['yaws'], dtype=float) if result['yaws'] else np.empty(0),
            'valid_masks': np.asarray(result['valid_masks'], dtype=np.uint32) if result['valid_masks'] else np.empty(0, dtype=np.uint32),
        }

    @staticmethod
    def _deduplicate_planner_trajectories(windows: List[Dict]) -> List[Dict]:
        """按 trajectory_id 和绝对期望时刻去除 rolling-horizon 重叠点。"""
        trajectories = {}
        for window in windows:
            trajectory_id = int(window['trajectory_id'])
            trajectory = trajectories.setdefault(trajectory_id, {
                'trajectory_id': trajectory_id,
                'points_by_time': {},
                'message_times': [],
                'is_horizon': False,
            })
            trajectory['message_times'].append(float(window['message_time']))
            trajectory['is_horizon'] = trajectory['is_horizon'] or bool(window['is_horizon'])
            for point in window['points']:
                point_key = round(float(point['time']), 9)
                trajectory['points_by_time'][point_key] = point

        result = []
        for trajectory in trajectories.values():
            points = [
                trajectory['points_by_time'][key]
                for key in sorted(trajectory['points_by_time'])
            ]
            result.append({
                'trajectory_id': trajectory['trajectory_id'],
                'is_horizon': trajectory['is_horizon'],
                'message_times': np.asarray(trajectory['message_times'], dtype=float),
                'points': points,
            })
        return sorted(result, key=lambda item: item['trajectory_id'])

    @staticmethod
    def _quat_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """四元数转欧拉角 (roll, pitch, yaw)"""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def _rotate_vector_by_quaternion(
        w: float, x: float, y: float, z: float, vector: np.ndarray,
    ) -> np.ndarray:
        """把 child/body frame 向量旋转到 odometry pose 的世界坐标系。"""
        quaternion = np.asarray([w, x, y, z], dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-12:
            return np.asarray(vector, dtype=float)
        w, x, y, z = quaternion / norm
        rotation = np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ])
        return rotation.dot(np.asarray(vector, dtype=float))

    def _extract_task_phase(
        self,
        data: Dict,
    ) -> Dict:
        """Extract the task evaluation interval."""
        analysis_phase = self.task.get_analysis_phase()
        analysis_state = analysis_phase.state_code
        states = data['flight_states']
        state_times = data['flight_state_times']

        if len(states) == 0:
            raise ValueError('bag 缺少必需话题 /flight_state，无法确定任务执行阶段。')

        emergency_mask = states == 6
        emergency_occurred = bool(np.any(emergency_mask))
        emergency_time = float(state_times[emergency_mask][0]) if emergency_occurred else 0.0

        mission_mask = states == analysis_state
        if not np.any(mission_mask):
            takeoff_mask = states == 2
            if np.any(takeoff_mask):
                last_takeoff_idx = np.where(takeoff_mask)[0][-1]
                mission_start = float(state_times[last_takeoff_idx])
                end_time = emergency_time if emergency_occurred else float(state_times[-1])
            else:
                mission_start = float(state_times[0])
                end_time = emergency_time if emergency_occurred else state_times[-1]
        else:
            mission_start_idx = int(np.where(mission_mask)[0][0])
            mission_start = float(state_times[mission_start_idx])
            mission_end_idx = mission_start_idx + 1
            while mission_end_idx < len(states) and states[mission_end_idx] == analysis_state:
                mission_end_idx += 1

            if mission_end_idx < len(state_times):
                mission_end_raw = state_times[mission_end_idx]
            else:
                mission_end_raw = state_times[-1]

            if emergency_occurred and emergency_time >= mission_start:
                end_time = min(mission_end_raw, emergency_time)
            else:
                end_time = mission_end_raw

        start_time = self._resolve_execution_start_time(data, mission_start, end_time)
        execution_started = start_time < end_time

        result = {}

        if len(data['position_times']) > 0:
            mask = (data['position_times'] >= start_time) & (data['position_times'] <= end_time)
            result['positions'] = data['positions'][mask]
            result['position_times'] = data['position_times'][mask]
            result['position_frames'] = data['position_frames'][mask]
        else:
            result['positions'] = np.empty((0, 3))
            result['position_times'] = np.empty(0)
            result['position_frames'] = np.empty(0, dtype=object)

        if len(data['velocity_times']) > 0:
            mask = (data['velocity_times'] >= start_time) & (data['velocity_times'] <= end_time)
            result['velocities'] = data['velocities'][mask]
            result['velocity_times'] = data['velocity_times'][mask]
        else:
            result['velocities'] = np.empty((0, 3))
            result['velocity_times'] = np.empty(0)

        if len(data['attitude_times']) > 0:
            mask = (data['attitude_times'] >= start_time) & (data['attitude_times'] <= end_time)
            result['attitudes'] = data['attitudes'][mask]
            result['attitude_times'] = data['attitude_times'][mask]
        else:
            result['attitudes'] = np.empty((0, 3))
            result['attitude_times'] = np.empty(0)

        if len(data['thrust_times']) > 0:
            mask = (data['thrust_times'] >= start_time) & (data['thrust_times'] <= end_time)
            result['thrusts'] = data['thrusts'][mask]
            result['thrust_times'] = data['thrust_times'][mask]
        else:
            result['thrusts'] = np.empty(0)
            result['thrust_times'] = np.empty(0)

        if len(data['ref_pos_times']) > 0:
            mask = (data['ref_pos_times'] >= start_time) & (data['ref_pos_times'] <= end_time)
            result['reference_positions'] = data['reference_positions'][mask]
            result['ref_pos_times'] = data['ref_pos_times'][mask]
            result['reference_velocities'] = data['reference_velocities'][mask]
            result['reference_accelerations'] = data['reference_accelerations'][mask]
            result['reference_jerks'] = data['reference_jerks'][mask]
            result['reference_yaws'] = data['reference_yaws'][mask]
            result['reference_valid_masks'] = data['reference_valid_masks'][mask]
        else:
            result['reference_positions'] = np.empty((0, 3))
            result['ref_pos_times'] = np.empty(0)
            result['reference_velocities'] = np.empty((0, 3))
            result['reference_accelerations'] = np.empty((0, 3))
            result['reference_jerks'] = np.empty((0, 3))
            result['reference_yaws'] = np.empty(0)
            result['reference_valid_masks'] = np.empty(0, dtype=np.uint32)

        if len(data['planner_output_times']) > 0:
            mask = (data['planner_output_times'] >= start_time) & (data['planner_output_times'] <= end_time)
            result['planner_output_positions'] = data['planner_output_positions'][mask]
            result['planner_output_times'] = data['planner_output_times'][mask]
        else:
            result['planner_output_positions'] = np.empty((0, 3))
            result['planner_output_times'] = np.empty(0)

        message_times = data['planner_output_message_times']
        if len(message_times) > 0:
            result['planner_output_message_times'] = message_times[
                (message_times >= start_time) & (message_times <= end_time)
            ]
        else:
            result['planner_output_message_times'] = np.empty(0)
        result['planner_output_message_count'] = int(len(result['planner_output_message_times']))
        point_message_times = data['planner_output_point_message_times']
        result['planner_output_point_count'] = int(np.count_nonzero(
            (point_message_times >= start_time) & (point_message_times <= end_time)
        )) if len(point_message_times) else 0
        phase_windows = [
            window for window in data.get('planner_output_windows', [])
            if start_time <= float(window['message_time']) <= end_time
        ]
        result['planner_trajectories'] = self._deduplicate_planner_trajectories(phase_windows)

        result['start_time'] = start_time
        result['end_time'] = end_time
        result['phase_name'] = analysis_phase.name
        result['phase_state'] = analysis_state
        result['emergency_occurred'] = emergency_occurred
        result['emergency_time'] = emergency_time
        result['mission_start_time'] = mission_start
        result['execution_started'] = execution_started

        return result

    def _first_time_after(self, times: np.ndarray, threshold: float) -> Optional[float]:
        if len(times) == 0:
            return None
        valid = times[times >= threshold]
        if len(valid) == 0:
            return None
        return float(valid[0])

    def _resolve_execution_start_time(
        self,
        data: Dict,
        mission_start: float,
        end_time: float,
    ) -> float:
        policy = self.task.execution_start_policy
        if policy == 'mission_state':
            return mission_start

        if policy == 'first_reference_or_planner_output':
            candidates = [
                self._first_time_after(data.get('planner_output_message_times', np.empty(0)), mission_start),
            ]
        elif policy == 'first_task_signal':
            candidates = [
                self._first_time_after(data.get('trigger_times', np.empty(0)), mission_start),
                self._first_time_after(data.get('waypoint_times', np.empty(0)), mission_start),
                self._first_time_after(data.get('planner_output_message_times', np.empty(0)), mission_start),
            ]
        else:
            candidates = [mission_start]

        valid_candidates = [
            candidate for candidate in candidates
            if candidate is not None and candidate < end_time
        ]
        if not valid_candidates:
            return end_time
        return min(valid_candidates)

    def _resolve_run_status(self) -> Dict[str, str]:
        stored = self.run_metadata.get('run_status')
        if isinstance(stored, dict) and stored.get('status'):
            return {
                'status': str(stored['status']),
                'reason': str(stored.get('reason', '')),
            }

        return {
            'status': 'unknown',
            'reason': 'metadata_unavailable',
        }

    def _derive_task_outcome(self, data: Dict, phase_data: Dict) -> TaskOutcome:
        stored = self.run_metadata.get('task_outcome')
        metadata_matches_task = self.run_metadata.get('task', self.task.name) == self.task.name
        if (
            not self.recompute_outcome
            and metadata_matches_task
            and isinstance(stored, dict)
            and stored.get('status')
        ):
            try:
                return TaskOutcome.from_dict(stored)
            except (TypeError, ValueError):
                pass

        evaluator = self.task.create_outcome_evaluator()
        if not self.task.has_terminal_outcome:
            return evaluator.outcome
        if not phase_data.get('execution_started', False):
            return evaluator.mark_unknown('execution_not_started')

        start_time = float(phase_data['start_time'])
        end_time = float(phase_data['end_time'])
        paths = [
            path for path in data.get('mission_paths', [])
            if float(path.get('time', 0.0)) <= end_time
        ]
        positions = phase_data.get('positions', np.empty((0, 3)))
        position_times = phase_data.get('position_times', np.empty(0))
        position_frames = phase_data.get('position_frames', np.empty(0, dtype=object))
        velocities = phase_data.get('velocities', np.empty((0, 3)))
        if len(position_frames) != len(position_times):
            position_frames = np.array([''] * len(position_times), dtype=object)

        earlier_paths = [
            path for path in paths if float(path.get('time', 0.0)) <= start_time
        ]
        if earlier_paths:
            latest = max(earlier_paths, key=lambda path: float(path.get('time', 0.0)))
            evaluator.update_path(
                latest.get('positions', []),
                start_time,
                frame_id=str(latest.get('frame_id', '') or ''),
            )
        evaluator.start(start_time)
        earlier_states = [
            state for state in data.get('planner_mission_states', [])
            if float(state.get('message_time', 0.0)) <= start_time
        ]
        if earlier_states and hasattr(evaluator, 'notify_mission_state'):
            latest_state = max(
                earlier_states,
                key=lambda state: float(state.get('message_time', 0.0)),
            )
            evaluator.notify_mission_state(
                start_time,
                mission_type=int(latest_state.get('mission_type', 0) or 0),
                status=int(latest_state.get('status', 0) or 0),
                completed_items=int(latest_state.get('completed_items', 0) or 0),
                total_items=int(latest_state.get('total_items', 0) or 0),
                detail=str(latest_state.get('detail', '') or ''),
            )

        events = [
            (
                float(path.get('time', 0.0)),
                0,
                (
                    path.get('positions', []),
                    str(path.get('frame_id', '') or ''),
                ),
            )
            for path in paths
            if float(path.get('time', 0.0)) > start_time
        ]
        events.extend(
            (
                float(stamp), 1,
                (
                    position,
                    str(frame_id or ''),
                    float(np.linalg.norm(velocity)),
                ),
            )
            for stamp, position, frame_id, velocity in zip(
                position_times,
                positions,
                position_frames,
                velocities,
            )
        )
        if hasattr(evaluator, 'notify_mission_state'):
            for state in data.get('planner_mission_states', []):
                stamp = float(state.get('message_time', 0.0))
                if stamp <= start_time or stamp > end_time:
                    continue
                events.append((stamp, 2, (
                    int(state.get('mission_type', 0) or 0),
                    int(state.get('status', 0) or 0),
                    int(state.get('completed_items', 0) or 0),
                    int(state.get('total_items', 0) or 0),
                    str(state.get('detail', '') or ''),
                )))
        events.sort(key=lambda item: (item[0], item[1]))

        completion_count = 0
        for stamp, kind, payload in events:
            if kind == 0:
                evaluator.update_path(
                    payload[0], stamp, frame_id=payload[1],
                )
            elif kind == 2:
                evaluator.notify_mission_state(
                    stamp,
                    mission_type=payload[0],
                    status=payload[1],
                    completed_items=payload[2],
                    total_items=payload[3],
                    detail=payload[4],
                )
                completion_count += 1
            else:
                evaluator.update_position(
                    payload[0], stamp, frame_id=payload[1], speed=payload[2],
                )
            if evaluator.terminal:
                break
        self._last_completion_count = completion_count

        if phase_data.get('emergency_occurred', False):
            return evaluator.emergency(end_time)

        execution_duration = max(0.0, end_time - start_time)
        run_status = self._resolve_run_status().get('status')
        if (
            run_status == 'completed'
            or execution_duration >= self.task.duration * 0.95
        ):
            return evaluator.timeout(end_time)
        return evaluator.mark_unknown('recording_ended_before_deadline')

    def analyze(self) -> AnalysisReport:
        """Analyze the bag and build a report."""
        data = self._extract_bag_data()
        hover_data = self._extract_task_phase(data)
        self._last_full_data = data
        self._last_phase_data = hover_data

        if self.hover_height is None:
            if len(data['reference_positions']) > 0:
                self.hover_height = float(np.mean(data['reference_positions'][:, 2]))
            elif len(data['planner_output_positions']) > 0:
                self.hover_height = float(np.mean(data['planner_output_positions'][:, 2]))
            elif len(hover_data['positions']) > 0:
                self.hover_height = float(np.mean(hover_data['positions'][:, 2]))
            else:
                self.hover_height = 2.0  # 默认

        self.task.takeoff_height = self.hover_height
        self.calculator.hover_height = self.hover_height

        if len(data['position_times']) > 1:
            flight_duration = float(data['position_times'][-1] - data['position_times'][0])
        else:
            flight_duration = 0.0

        execution_duration = float(
            hover_data.get('end_time', 0) - hover_data.get('start_time', 0)
        )

        run_status = self._resolve_run_status()
        task_outcome = self._derive_task_outcome(data, hover_data)

        metrics = self.task.compute_metrics(self.calculator, hover_data, data)
        metrics.extend(self._build_system_metrics(hover_data, task_outcome))
        summary = self._generate_summary(metrics, run_status, task_outcome, hover_data)


        emergency_occurred = hover_data.get('emergency_occurred', False)
        emergency_detail = {}
        if emergency_occurred:
            emergency_detail = {
                'triggered': True,
                'trigger_time': hover_data.get('emergency_time', 0),
            }
            if len(data['flight_state_times']) > 0 and len(data['flight_states']) > 0:
                em_mask = data['flight_states'] == 6
                if np.any(em_mask):
                    emergency_detail['trigger_time'] = float(data['flight_state_times'][em_mask][0])
            if emergency_detail.get('trigger_time', 0) > 0:
                prev_mask = data['flight_state_times'] < emergency_detail['trigger_time']
                if np.any(prev_mask):
                    prev_states = data['flight_states'][prev_mask]
                    emergency_detail['prev_state'] = FLIGHT_STATE_MAP.get(int(prev_states[-1]), str(prev_states[-1]))

        return AnalysisReport(
            controller=self.controller_name,
            task=self.task.name,
            bag_file=self.bag_file,
            hover_height=self.hover_height,
            flight_duration=flight_duration,
            execution_duration=max(execution_duration, 0),
            metric_profile=self.task.metric_profile,
            analysis_phase=hover_data.get('phase_name', ''),
            run_status=run_status,
            task_outcome=task_outcome.to_dict(),
            metrics=metrics,
            summary=summary,
            emergency_occurred=emergency_occurred,
            emergency_detail=emergency_detail,
            artifacts={},
        )

    def get_cached_data(self) -> Tuple[Dict, Dict]:
        """Return data cached by the latest analysis."""
        if self._last_full_data is None or self._last_phase_data is None:
            raise RuntimeError('请先调用 analyze()，再获取缓存的 bag 数据。')
        return self._last_full_data, self._last_phase_data

    def _build_system_metrics(
        self,
        phase_data: Dict,
        task_outcome: TaskOutcome,
    ) -> List[MetricResult]:
        """补充整链路系统级兜底指标。"""
        metrics = [
            self.calculator.compute_emergency_status(
                emergency_occurred=phase_data.get('emergency_occurred', False),
                emergency_time=phase_data.get('emergency_time', 0.0),
            ),
            MetricResult(
                name='execution_started',
                description='任务执行阶段已开始',
                value=1.0 if phase_data.get('execution_started', False) else 0.0,
                unit='bool',
                group='outcome',
                source='/flight_state + task start signal',
                detail={
                    'started': bool(phase_data.get('execution_started', False)),
                    'mission_start_time': float(phase_data.get('mission_start_time', 0.0)),
                    'execution_start_time': float(phase_data.get('start_time', 0.0)),
                },
            ),
        ]
        evidence = task_outcome.evidence
        completion_events = int(evidence.get('planner_state_updates', 0) or 0)
        metrics.append(MetricResult(
            name='planner_mission_state_count',
            description='规划器上报任务状态次数',
            value=float(completion_events),
            unit='count',
            group='outcome',
            source='/planner/mission_state',
            detail={
                'rejected_state_count': evidence.get('rejected_planner_state_updates'),
                'last_state_time': evidence.get('last_planner_state_time'),
            },
        ))
        collision_summary = self.run_metadata.get('collision_summary', {})
        if isinstance(collision_summary, dict):
            collision_count = collision_summary.get('episode_count')
            if collision_count is not None:
                limit_reached = bool(collision_summary.get('limit_reached', False))
                metrics.append(MetricResult(
                    name='collision_episodes',
                    description='任务执行阶段独立碰撞次数',
                    value=float(collision_count),
                    unit='count',
                    group='outcome',
                    source='/flight_eval/contact_pulse',
                    detail={
                        'episode_limit': collision_summary.get('episode_limit'),
                        'episode_gap': collision_summary.get('episode_gap'),
                        'limit_reached': limit_reached,
                        'pair_counts': collision_summary.get('pair_counts', {}),
                    },
                ))
        if task_outcome.completion_time is not None:
            metrics.append(MetricResult(
                name='completion_time',
                description='整体任务完成时间',
                value=float(task_outcome.completion_time),
                unit='s',
                group='outcome',
                source='task outcome evaluator',
                detail={'outcome_reason': task_outcome.reason},
            ))
        final_distance = evidence.get('final_goal_distance')
        if final_distance is not None:
            mission_detection = (
                'planner_mission_state'
                if completion_events > 0
                else 'none'
            )
            metrics.append(MetricResult(
                name='final_goal_distance',
                description='任务结束时距最终目标距离',
                value=float(final_distance),
                unit='m',
                group='navigation',
                source=f'{MISSION_PATH_TOPIC} + odometry',
                detail={
                    'goal_tolerance': evidence.get('goal_tolerance'),
                    'remaining_path': evidence.get('remaining_path'),
                    'mission_detection': mission_detection,
                },
            ))
        return metrics

    def _generate_summary(
        self,
        metrics: List[MetricResult],
        run_status: Dict[str, str],
        task_outcome: TaskOutcome,
        hover_data: Optional[Dict] = None,
    ) -> str:
        """生成摘要文字"""
        lines = [
            f"  run_status: {run_status.get('status', 'unknown')}",
            f"  task_outcome: {task_outcome.status.value} ({task_outcome.reason})",
        ]

        if hover_data and hover_data.get('emergency_occurred', False):
            em_time = hover_data.get('emergency_time', 0)
            lines.append(f"  EMERGENCY 触发 @ {em_time:.1f}s")

        groups = {}
        for metric in metrics:
            groups.setdefault(metric.group, []).append(metric)
        for group, group_metrics in groups.items():
            lines.append(f"  {GROUP_LABELS.get(group, group)}:")
            for metric in group_metrics:
                lines.append(f"    {metric.description}: {self._format_metric_value(metric)}")

        if not lines:
            return "数据不足, 无法计算指标"

        return "\n".join(lines)

    def save_report(self, report: AnalysisReport, output_dir: str) -> str:
        """Save JSON and Markdown reports."""
        if output_dir.lower().endswith('.json'):
            filepath = output_dir
            output_dir = os.path.dirname(filepath) or '.'
        else:
            filepath = os.path.join(output_dir, REPORT_FILENAME)

        os.makedirs(output_dir, exist_ok=True)
        report_dict = {
            'format_version': 2,
            'controller': report.controller,
            'task': report.task,
            'metric_profile': report.metric_profile,
            'bag_file': report.bag_file,
            'hover_height': report.hover_height,
            'flight_duration': report.flight_duration,
            'execution_duration': report.execution_duration,
            'analysis_phase': report.analysis_phase,
            'run_status': report.run_status,
            'task_outcome': report.task_outcome,
            'emergency': {
                'occurred': report.emergency_occurred,
                'detail': report.emergency_detail,
            },
            'artifacts': report.artifacts,
            'summary': report.summary,
            'metrics': [
                {
                    'name': m.name,
                    'description': m.description,
                    'value': m.value if m.available else None,
                    'unit': m.unit,
                    'group': m.group,
                    'source': m.source,
                    'phase': m.phase or report.analysis_phase,
                    'available': m.available,
                    'detail': {k: (v if not isinstance(v, float) or not math.isnan(v) else None)
                               for k, v in m.detail.items()},
                }
                for m in report.metrics
            ],
        }

        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(report_dict, f, indent=2, ensure_ascii=False, cls=NumpyEncoder)

        summary_path = os.path.join(output_dir, AGENT_SUMMARY_FILENAME)
        self._save_agent_summary(report, summary_path)

        return filepath

    @staticmethod
    def _format_metric_value(metric: MetricResult) -> str:
        """把指标值格式化为适合 Markdown 表格阅读的文本。"""
        if not metric.available:
            return 'N/A'
        if metric.name == 'emergency_status':
            return 'triggered' if metric.detail.get('triggered', False) else 'not triggered'
        value = f'{metric.value:.4f}'
        return f'{value} {metric.unit}'.strip()

    def _save_agent_summary(self, report: AnalysisReport, filepath: str) -> str:
        """保存单次运行的紧凑 Markdown 摘要。"""
        output_dir = os.path.dirname(filepath) or '.'
        run_name = os.path.basename(os.path.abspath(output_dir))
        parameter_manifest_path = os.path.join(output_dir, PARAMETER_SNAPSHOT_MANIFEST_FILENAME)
        rosparam_snapshot_path = os.path.join(output_dir, ROSPARAM_SNAPSHOT_FILENAME)

        lines = [
            '# Flight Evaluation Summary',
            '',
            f'- Run: `{run_name}`',
            f'- Controller: `{report.controller}`',
            f'- Task: `{report.task}`',
            f'- Metric profile: `{report.metric_profile}`',
            f'- Bag: `{os.path.basename(report.bag_file)}`',
            f'- Flight duration: {report.flight_duration:.2f} s',
            f'- Analysis phase: `{report.analysis_phase or "N/A"}` '
            f'({report.execution_duration:.2f} s)',
            f'- Run status: `{report.run_status.get("status", "unknown")}` '
            f'({report.run_status.get("reason", "")})',
            f'- Task outcome: `{report.task_outcome.get("status", "unknown")}` '
            f'({report.task_outcome.get("reason", "")})',
            f'- Hover height: {report.hover_height:.3f} m',
            f'- Emergency: {"yes" if report.emergency_occurred else "no"}',
            '',
            '## Effective parameters',
            '',
        ]

        if os.path.isfile(parameter_manifest_path):
            with open(parameter_manifest_path, 'r', encoding='utf-8') as manifest_file:
                manifest = json.load(manifest_file)

            rosparam_info = manifest.get('rosparam_snapshot', {})
            rosparam_file = rosparam_info.get('rosparam_snapshot_file', '')
            if rosparam_file:
                lines.append(f'- Runtime parameter snapshot: `{rosparam_file}`')
            elif os.path.isfile(rosparam_snapshot_path):
                lines.append(f'- Runtime parameter snapshot: `{ROSPARAM_SNAPSHOT_FILENAME}`')

            targets = rosparam_info.get('targets', {})
            if targets.get('node_namespaces'):
                lines.append(
                    '- ROS node parameter namespaces: `'
                    + '`, `'.join(targets['node_namespaces'])
                    + '`'
                )
            if targets.get('declared_parameters'):
                lines.append(
                    '- Additional launch parameters: `'
                    + '`, `'.join(targets['declared_parameters'])
                    + '`'
                )
            for error in rosparam_info.get('errors', []):
                lines.append(f'- Parameter snapshot error: `{error}`')

            planner_info = manifest.get('planner', {})
            launch_args = planner_info.get('launch_args', {})
            requested_args = planner_info.get('requested_args', {})
            if launch_args:
                lines.extend([
                    '',
                    f'- Planner launch args: `{json.dumps(launch_args, ensure_ascii=False, sort_keys=True)}`',
                ])
            if requested_args:
                lines.append(
                    f'- Requested planner args: `{json.dumps(requested_args, ensure_ascii=False, sort_keys=True)}`'
                )

            algorithm_manifests = manifest.get('algorithm_manifests', [])
            if algorithm_manifests:
                lines.extend([
                    '',
                    'Algorithm manifests:',
                ])
                for path in algorithm_manifests:
                    lines.append(f'- `{path}`')
            else:
                lines.append('- Algorithm manifests: none (built-in algorithms)')

        else:
            lines.append(
                f'Parameter snapshot unavailable: `{PARAMETER_SNAPSHOT_MANIFEST_FILENAME}` '
                'was not found beside the bag.'
            )

        lines.extend([
            '',
            '## Metrics',
            '',
            '| Metric | Group | Source | Value | Available |',
            '|---|---|---|---:|---|',
        ])
        for metric in report.metrics:
            description = metric.description.replace('|', '\\|')
            lines.append(
                f'| `{metric.name}` — {description} | {metric.group} | '
                f'{metric.source} | {self._format_metric_value(metric)} | '
                f'{"yes" if metric.available else "no"} |'
            )

        if report.artifacts:
            lines.extend(['', '## Artifacts', ''])
            for name, path in report.artifacts.items():
                display_path = os.path.relpath(path, output_dir) if os.path.isabs(path) else path
                lines.append(f'- `{name}`: `{display_path}`')

        lines.append('')
        with open(filepath, 'w', encoding='utf-8') as summary_file:
            summary_file.write('\n'.join(lines))
        return filepath

    def print_report(self, report: AnalysisReport):
        """打印分析报告到终端"""
        print("\n" + "=" * 70)
        print(f"  飞行评估报告 - {report.controller} - {report.task}")
        print("=" * 70)
        print(f"  Bag 文件:     {report.bag_file}")
        print(f"  指标集合:     {report.metric_profile}")
        print(f"  悬停高度:     {report.hover_height:.2f} m")
        print(f"  完整飞行时长:     {report.flight_duration:.1f} s")
        print(f"  执行阶段时长: {report.execution_duration:.1f} s")
        print(
            f"  运行状态:     {report.run_status.get('status', 'unknown')} "
            f"({report.run_status.get('reason', '')})"
        )
        print(
            f"  任务结果:     {report.task_outcome.get('status', 'unknown')} "
            f"({report.task_outcome.get('reason', '')})"
        )
        if report.task_outcome.get('completion_time') is not None:
            print(f"  完成时间:     {report.task_outcome['completion_time']:.1f} s")
        if report.analysis_phase:
            print(f"  分析阶段:     {report.analysis_phase}")
        if report.artifacts:
            print(f"  可视化产物:   {report.artifacts}")
        print("-" * 70)
        print("  指标框架:")
        print()

        if report.emergency_occurred:
            em = report.emergency_detail
            trigger_t = em.get('trigger_time', 0)
            prev_st = em.get('prev_state', 'N/A')
            print(f"  [⚠ 紧急状态]")
            print(f"    触发时间: {trigger_t:.1f}s")
            print(f"    触发前状态: {prev_st}")
            print(f"    以下指标基于紧急触发前的稳态数据")
            print()

        grouped = {}
        for metric in report.metrics:
            grouped.setdefault(metric.group, []).append(metric)
        for group, metrics in grouped.items():
            print(f"  [{GROUP_LABELS.get(group, group)}]")
            for metric in metrics:
                value = self._format_metric_value(metric)
                print(f"    {metric.description}: {value}")
                print(f"      source: {metric.source}")
                if not metric.available and metric.detail.get('error'):
                    print(f"      reason: {metric.detail['error']}")
            print()
        print("=" * 70)
