#!/usr/bin/env python3
"""Run one flight evaluation lifecycle."""

import os
import json
import math
import shutil
import signal
import subprocess
import time
import threading
from enum import Enum
from typing import Dict, List, Optional

from .controllers import CONTROLLER_REGISTRY, get_controller_launch
from .launch_inspection import inspect_launch, is_within_namespace
from .outcomes import MISSION_PATH_TOPIC
from .planners import PLANNER_REGISTRY, get_planner_launch
from .tasks import create_task


BASE_RECORD_TOPICS = [
    '/mavros/local_position/odom',
    '/mavros/state',
    '/flight_eval/contact_pulse',
]
CONTACT_PULSE_TOPIC = '/flight_eval/contact_pulse'
TASK_CLOCK_STALL_TIMEOUT = 30.0

_OPENDRONE_PKG_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
_OPENDRONE_LAUNCH_ROOT = os.path.join(_OPENDRONE_PKG_ROOT, 'launch')
ROSPARAM_SNAPSHOT_FILENAME = 'rosparams.json'
RUN_METADATA_FILENAME = 'run_metadata.json'


class RunInterrupted(Exception):
    """运行被 ROS shutdown 或用户中断取消。"""


class RunnerState(str, Enum):
    """一次 flight_eval 运行的编排状态，不等同于 controller 的 flight_state。"""

    PREPARING = 'preparing'
    RECORDING = 'recording'
    WAITING_FOR_CONTROLLER = 'waiting_for_controller'
    STARTING_PLANNER = 'starting_planner'
    WAITING_FOR_PLANNER = 'waiting_for_planner'
    EXECUTING = 'executing'
    REQUESTING_LAND = 'requesting_land'
    WAITING_FOR_LANDED = 'waiting_for_landed'
    CAPTURING_ARTIFACTS = 'capturing_artifacts'
    ABORTING = 'aborting'
    COMPLETED = 'completed'
    FAILED = 'failed'
    INTERRUPTED = 'interrupted'


_RUNNER_TRANSITIONS = {
    RunnerState.PREPARING: {RunnerState.RECORDING},
    RunnerState.RECORDING: {RunnerState.WAITING_FOR_CONTROLLER},
    RunnerState.WAITING_FOR_CONTROLLER: {RunnerState.STARTING_PLANNER},
    RunnerState.STARTING_PLANNER: {
        RunnerState.WAITING_FOR_PLANNER,
        RunnerState.EXECUTING,
    },
    RunnerState.WAITING_FOR_PLANNER: {RunnerState.EXECUTING},
    RunnerState.EXECUTING: {
        RunnerState.REQUESTING_LAND,
        RunnerState.CAPTURING_ARTIFACTS,
    },
    RunnerState.REQUESTING_LAND: {RunnerState.WAITING_FOR_LANDED},
    RunnerState.WAITING_FOR_LANDED: {RunnerState.CAPTURING_ARTIFACTS},
    RunnerState.ABORTING: {RunnerState.CAPTURING_ARTIFACTS},
    RunnerState.CAPTURING_ARTIFACTS: {RunnerState.COMPLETED},
    RunnerState.COMPLETED: set(),
    RunnerState.FAILED: set(),
    RunnerState.INTERRUPTED: set(),
}


class FlightRunner:
    """飞行评估运行器"""

    def __init__(
        self,
        controller_name: str,
        task_name: str = 'hover',
        planner_name: str = 'none',
        planner_args: Optional[Dict[str, str]] = None,
        controller_args: Optional[Dict[str, str]] = None,
        takeoff_height: float = 2.0,
        duration: Optional[float] = None,
        record_dir: Optional[str] = None,
        auto_land: bool = True,
        extra_topics: Optional[List[str]] = None,
        metadata_extra: Optional[Dict[str, object]] = None,
        max_collision_episodes: int = 3,
        collision_episode_gap: float = 0.5,
    ):
        self._rosbag_process = None
        self._controller_process = None
        self._planner_process = None
        self._cleaned_up = False
        self._cleanup_report = {
            'completed': None,
            'registered_process_groups': [],
            'remaining_process_groups': [],
        }
        self._archived_manifest_files = []
        self._started_launches = []

        self.controller_name = controller_name
        self.task = create_task(
            task_name,
            takeoff_height=takeoff_height,
            duration=duration,
        )
        self.task_name = self.task.name
        self.requested_planner_name = planner_name
        self.planner_name = planner_name
        self.planner_args = dict(self.task.planner_args)
        self.planner_args.update(planner_args or {})
        if self.task.waypoint_type:
            self.planner_args.setdefault('waypoint_type', self.task.waypoint_type)
        if self.planner_name == 'none' and self.task.planner_name_override:
            self.planner_name = self.task.planner_name_override
            merged_args = dict(self.task.planner_args)
            merged_args.update(self.planner_args)
            self.planner_args = merged_args
        self.takeoff_height = self.task.takeoff_height
        self.duration = self.task.duration
        self.record_dir = os.path.abspath(record_dir) if record_dir else os.path.abspath('./eval_runs')
        self.auto_land = auto_land
        self.extra_topics = extra_topics or []
        self.metadata_extra = dict(metadata_extra or {})
        if (
            not isinstance(max_collision_episodes, int)
            or isinstance(max_collision_episodes, bool)
            or max_collision_episodes < 1
        ):
            raise ValueError('max_collision_episodes 必须是正整数')
        if collision_episode_gap <= 0.0:
            raise ValueError('collision_episode_gap 必须大于 0')
        self.max_collision_episodes = int(max_collision_episodes)
        self.collision_episode_gap = float(collision_episode_gap)

        if controller_name not in CONTROLLER_REGISTRY:
            raise ValueError(
                f"未知控制器: {controller_name}. "
                f"可用: {list(CONTROLLER_REGISTRY.keys())}"
            )

        if self.planner_name not in PLANNER_REGISTRY:
            raise ValueError(
                f"未知规划器: {self.planner_name}. "
                f"可用: {list(PLANNER_REGISTRY.keys())}"
            )

        self.controller_args = dict(controller_args or {})
        self.ctrl_info = get_controller_launch(controller_name, self.controller_args)
        self.planner_info = get_planner_launch(self.planner_name, self.planner_args)
        self.controller_topics = self.ctrl_info['evaluation_topics']
        self.planner_topics = self.planner_info['evaluation_topics']
        self._validate_task_contracts()
        self._state_condition = threading.Condition()
        self._runner_state = RunnerState.PREPARING
        self._state_history = [{
            'state': self._runner_state.value,
            'time': time.time(),
            'reason': 'runner created',
        }]
        self._target_flight_state = self.task.target_state_code
        self._last_flight_state = None
        self._emergency_occurred = False
        self._planner_output_received = False
        self._latest_mission_state = None
        self._task_evaluator = self.task.create_outcome_evaluator()
        self._execution_end_reason = ''
        self._collision_episode_count = 0
        self._collision_limit_reached = False
        self._collision_last_seen = {}
        self._collision_pair_counts = {}
        self._collision_events = []
        self._task_time_source = time.monotonic
        self._task_clock_name = 'monotonic_fallback'

    def _task_now(self) -> float:
        return float(self._task_time_source())

    def _use_ros_task_clock(self, rospy) -> None:
        self._task_time_source = rospy.get_time
        self._task_clock_name = 'ros'

    def _validate_task_contracts(self):
        for kind, name, info in (
            ('控制器', self.controller_name, self.ctrl_info),
            ('规划器', self.planner_name, self.planner_info),
        ):
            allowed_tasks = info.get('tasks', [])
            if allowed_tasks and self.task.name not in allowed_tasks:
                raise ValueError(
                    f"{kind} {name} 不允许用于任务 {self.task.name}，"
                    f"可用: {allowed_tasks}"
                )

    @property
    def runner_state(self) -> RunnerState:
        """返回当前编排状态；供运行元数据和诊断使用。"""
        with self._state_condition:
            return self._runner_state

    def _transition(self, next_state: RunnerState, reason: str = ''):
        """执行唯一的生命周期状态迁移，并唤醒等待中的主流程。"""
        with self._state_condition:
            current_state = self._runner_state
            if current_state == next_state:
                return
            if current_state in {
                RunnerState.COMPLETED,
                RunnerState.FAILED,
                RunnerState.INTERRUPTED,
            }:
                return

            terminal = {
                RunnerState.ABORTING,
                RunnerState.FAILED,
                RunnerState.INTERRUPTED,
            }
            if next_state not in terminal:
                allowed = _RUNNER_TRANSITIONS[current_state]
                if next_state not in allowed:
                    raise RuntimeError(
                        f'非法 flight_eval 生命周期迁移: '
                        f'{current_state.value} -> {next_state.value}'
                    )

            self._runner_state = next_state
            self._state_history.append({
                'state': next_state.value,
                'time': time.time(),
                'reason': reason,
            })
            self._state_condition.notify_all()

    def _on_flight_state(self, state_code: int):
        """将 controller 状态事件映射为 Runner 生命周期迁移。"""
        with self._state_condition:
            self._last_flight_state = int(state_code)
            if state_code == 6:  # EMERGENCY
                self._emergency_occurred = True
                self._task_evaluator.emergency(self._task_now())
                self._transition(RunnerState.ABORTING, 'controller reported EMERGENCY')
            elif (
                self._runner_state == RunnerState.WAITING_FOR_CONTROLLER
                and state_code == self._target_flight_state
            ):
                self._transition(
                    RunnerState.STARTING_PLANNER,
                    f'controller reached flight_state={state_code}',
                )
            elif self._runner_state == RunnerState.WAITING_FOR_LANDED and state_code == 5:
                self._transition(RunnerState.CAPTURING_ARTIFACTS, 'controller reported LANDED')

    def _on_mission_path(self, msg):
        """向通用任务 evaluator 提交原始 mission Path。"""
        points = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in getattr(msg, 'poses', [])
        ]
        with self._state_condition:
            self._task_evaluator.update_path(
                points,
                self._task_now(),
                frame_id=getattr(msg.header, 'frame_id', ''),
            )
            self._state_condition.notify_all()

    def _on_odometry(self, msg):
        """Submit odometry as diagnostic evidence only."""
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        speed = math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )
        with self._state_condition:
            self._task_evaluator.update_position(
                (position.x, position.y, position.z),
                self._task_now(),
                frame_id=getattr(msg.header, 'frame_id', ''),
                speed=speed,
            )
            self._state_condition.notify_all()

    def _on_mission_state(self, msg):
        """Consume planner-owned mission state without interpreting geometry."""
        with self._state_condition:
            state = {
                'mission_type': int(msg.mission_type),
                'status': int(msg.status),
                'completed_items': int(msg.completed_items),
                'total_items': int(msg.total_items),
                'detail': str(msg.detail or ''),
            }
            self._latest_mission_state = state
            if (
                self._runner_state == RunnerState.EXECUTING
                and hasattr(self._task_evaluator, 'notify_mission_state')
            ):
                self._task_evaluator.notify_mission_state(
                    self._task_now(),
                    **state,
                )
                self._state_condition.notify_all()

    def _on_contact_pulse(self, msg, now: Optional[float] = None):
        """累计碰撞事件；连续 contact 心跳不重复计数。"""
        received_at = time.monotonic() if now is None else float(now)
        self_collision = str(getattr(msg, 'self_collision', '') or '')
        other_collision = str(getattr(msg, 'other_collision', '') or '')
        if not other_collision or 'ground_plane' in other_collision:
            return

        episode_key = other_collision
        with self._state_condition:
            if self._runner_state != RunnerState.EXECUTING:
                return
            if self.task.has_terminal_outcome and self._task_evaluator.terminal:
                return
            previous = self._collision_last_seen.get(episode_key)
            self._collision_last_seen[episode_key] = received_at
            if (
                previous is not None
                and received_at - previous <= self.collision_episode_gap
            ):
                return

            self._collision_episode_count += 1
            self._collision_pair_counts[episode_key] = (
                self._collision_pair_counts.get(episode_key, 0) + 1
            )
            self._collision_events.append({
                'episode': self._collision_episode_count,
                'time': received_at,
                'self_collision': self_collision,
                'other_collision': other_collision,
                'point_count': int(getattr(msg, 'point_count', 0) or 0),
            })
            if self._collision_episode_count < self.max_collision_episodes:
                self._state_condition.notify_all()
                return

            self._collision_limit_reached = True
            self._execution_end_reason = 'collision_limit'
            self._task_evaluator.collision_limit(
                received_at,
                self._collision_episode_count,
                self.max_collision_episodes,
            )
            self._transition(
                RunnerState.ABORTING,
                f'collision episode limit reached '
                f'({self._collision_episode_count}/{self.max_collision_episodes})',
            )

    @staticmethod
    def _planner_output_error(msg) -> str:
        if not msg.header.frame_id:
            return 'header.frame_id is empty'
        if msg.trajectory_start_time.to_sec() <= 0.0:
            return 'trajectory_start_time is not set'
        if not msg.points:
            return 'points is empty'
        previous_time = -1.0
        for index, point in enumerate(msg.points):
            if not (point.valid_mask & 1):
                return f'points[{index}] has no valid position'
            point_time = point.time_from_start.to_sec()
            if point_time < 0.0 or point_time < previous_time:
                return 'point times must be non-negative and ordered'
            previous_time = point_time
        return ''

    def _on_planner_output(self, msg, rospy):
        error = self._planner_output_error(msg)
        if error:
            rospy.logwarn_throttle(2.0, 'invalid PlannerOutput: %s', error)
            return
        with self._state_condition:
            self._planner_output_received = True
            if self._runner_state == RunnerState.WAITING_FOR_PLANNER:
                self._transition(RunnerState.EXECUTING, 'received first planner output')

    def _wait_for_state(
        self,
        expected_state: RunnerState,
        timeout: float,
        rospy,
        label: str,
        timeout_state: RunnerState = RunnerState.FAILED,
    ) -> bool:
        """等待状态机迁移，统一处理 emergency、ROS shutdown 与超时。"""
        deadline = time.monotonic() + timeout
        with self._state_condition:
            while self._runner_state != expected_state:
                if self._runner_state in {
                    RunnerState.ABORTING,
                    RunnerState.FAILED,
                    RunnerState.INTERRUPTED,
                }:
                    return False
                if rospy.is_shutdown():
                    raise RunInterrupted('ROS 已关闭')

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._transition(timeout_state, f'{label} timed out after {timeout:.1f}s')
                    return False
                self._state_condition.wait(timeout=min(remaining, 0.5))
        return True

    def _wait_for_task_end(self, rospy) -> str:
        """等待定时窗口结束或有限任务到达语义终点。"""
        deadline = self._task_now() + self.task.duration
        last_task_time = self._task_now()
        last_task_progress_wall = time.monotonic()
        with self._state_condition:
            while self._runner_state == RunnerState.EXECUTING:
                if rospy.is_shutdown():
                    raise RunInterrupted('收到 ROS shutdown')
                if self.task.has_terminal_outcome and self._task_evaluator.terminal:
                    return 'goal_reached'
                task_now = self._task_now()
                if task_now > last_task_time + 1e-9:
                    last_task_time = task_now
                    last_task_progress_wall = time.monotonic()
                elif (
                    time.monotonic() - last_task_progress_wall
                    >= TASK_CLOCK_STALL_TIMEOUT
                ):
                    self._task_evaluator.mark_unknown('task_clock_stalled')
                    return 'task_clock_stalled'
                remaining = deadline - task_now
                if remaining <= 0:
                    self._task_evaluator.timeout(task_now)
                    return 'timeout' if self.task.has_terminal_outcome else 'duration_elapsed'
                self._state_condition.wait(timeout=min(remaining, 0.5))
        return ''

    def _lifecycle_metadata(self) -> Dict[str, object]:
        with self._state_condition:
            return {
                'final_state': self._runner_state.value,
                'last_flight_state': self._last_flight_state,
                'transitions': list(self._state_history),
            }

    def _run_status_metadata(self) -> Dict[str, str]:
        with self._state_condition:
            final_state = self._runner_state
            reason = self._state_history[-1]['reason'] if self._state_history else ''
        status = {
            RunnerState.COMPLETED: 'completed',
            RunnerState.INTERRUPTED: 'interrupted',
        }.get(final_state, 'failed')
        if self._cleanup_report.get('completed') is False:
            status = 'failed'
            reason = 'process_cleanup_incomplete'
        return {
            'status': status,
            'reason': reason or final_state.value,
        }

    def _task_outcome_metadata(self) -> Dict[str, object]:
        return self._task_evaluator.outcome.to_dict()

    def _collision_metadata(self) -> Dict[str, object]:
        with self._state_condition:
            return {
                'episode_count': self._collision_episode_count,
                'episode_limit': self.max_collision_episodes,
                'episode_gap': self.collision_episode_gap,
                'limit_reached': self._collision_limit_reached,
                'pair_counts': dict(self._collision_pair_counts),
                'events': list(self._collision_events),
            }

    def _resolve_record_topics(self) -> List[str]:
        """根据控制器、规划器和任务契约组装录包话题。"""
        topics = (
            BASE_RECORD_TOPICS
            + self.ctrl_info['record_topics']
            + self.planner_info['record_topics']
            + self.task.record_topics
            + self.extra_topics
        )
        return list(dict.fromkeys(topics))

    def _capture_runtime_parameter_snapshot(self) -> Dict[str, object]:
        """保存本次实际启动的 launch 所属参数。"""
        snapshot_path = os.path.join(self.record_dir, ROSPARAM_SNAPSHOT_FILENAME)
        inspections = []
        errors = []
        for launch in self._started_launches:
            try:
                inspection = inspect_launch(
                    package=launch['package'],
                    launch_file=launch['launch_file'],
                    args=launch['args'],
                    resolved_path=launch['resolved_path'],
                )
                inspection['role'] = launch['role']
                inspections.append(inspection)
            except Exception as exc:
                errors.append(f"{launch['role']}: {exc}")

        node_namespaces = sorted({
            namespace
            for inspection in inspections
            for namespace in inspection['node_namespaces']
        })
        declared_parameters = sorted({
            parameter
            for inspection in inspections
            for parameter in inspection['declared_parameters']
            if not any(
                is_within_namespace(parameter, namespace)
                for namespace in node_namespaces
            )
        })
        targets = node_namespaces + declared_parameters
        parameters = {}
        missing = []
        try:
            import rospy
            for target in targets:
                try:
                    parameters[target] = rospy.get_param(target)
                except KeyError:
                    missing.append(target)
                except Exception as exc:
                    errors.append(f'{target}: {exc}')
        except ImportError as exc:
            errors.append(str(exc))

        archived_inspections = []
        for inspection in inspections:
            archived = dict(inspection)
            archived.pop('declared_parameters', None)
            archived_inspections.append(archived)

        runtime_expectations = dict(
            self.planner_info.get('runtime_parameter_expectations', {})
        )
        expectation_results = []

        def lookup_parameter(parameter_name):
            for namespace, namespace_value in parameters.items():
                if parameter_name == namespace:
                    return True, namespace_value
                prefix = namespace.rstrip('/') + '/'
                if not parameter_name.startswith(prefix):
                    continue
                value = namespace_value
                for component in parameter_name[len(prefix):].split('/'):
                    if not isinstance(value, dict) or component not in value:
                        return False, None
                    value = value[component]
                return True, value
            return False, None

        for parameter_name, expected_value in runtime_expectations.items():
            available, actual_value = lookup_parameter(parameter_name)
            matches = available and actual_value == expected_value
            expectation_results.append({
                'parameter': parameter_name,
                'expected': expected_value,
                'actual': actual_value if available else None,
                'matches': matches,
            })
            if not matches:
                errors.append(
                    'runtime parameter contract violation: '
                    f'{parameter_name} expected {expected_value!r}, '
                    f'got {actual_value!r}'
                )

        payload = {
            'format_version': 2,
            'parameters': parameters,
            'targets': {
                'node_namespaces': node_namespaces,
                'declared_parameters': declared_parameters,
            },
            'missing': missing,
            'launches': archived_inspections,
            'runtime_parameter_contract': expectation_results,
            'errors': errors,
        }
        try:
            with open(snapshot_path, 'w', encoding='utf-8') as snapshot_file:
                json.dump(payload, snapshot_file, indent=2, ensure_ascii=False, default=str)
            print(f"[snapshot] 参数服务器快照已保存: {snapshot_path}")
        except Exception as exc:
            print(f"[snapshot] 保存参数服务器快照失败: {exc}")
            snapshot_path = ''

        result = {
            'rosparam_snapshot_file': os.path.basename(snapshot_path) if snapshot_path else '',
            'targets': payload['targets'],
            'available': bool(parameters),
            'missing': missing,
            'errors': errors,
            'runtime_parameter_contract': expectation_results,
        }
        return result

    def _archive_manifests(self) -> None:
        sources = list(dict.fromkeys(
            path for path in (
                self.ctrl_info.get('manifest_path'),
                self.planner_info.get('manifest_path'),
            ) if path
        ))
        if not sources:
            return
        destination_dir = os.path.join(self.record_dir, 'algorithm_manifests')
        os.makedirs(destination_dir, exist_ok=True)
        for index, source in enumerate(sources, start=1):
            destination = os.path.join(
                destination_dir, f'{index:02d}_{os.path.basename(source)}'
            )
            shutil.copyfile(source, destination)
            self._archived_manifest_files.append(
                os.path.relpath(destination, self.record_dir)
            )

    def _start_rosbag(self) -> str:
        """启动 rosbag 录制并返回文件前缀。"""
        os.makedirs(self.record_dir, exist_ok=True)

        topics = self._resolve_record_topics()

        prefix = os.path.join(self.record_dir, 'flight_test')
        cmd = ['rosbag', 'record', '-O', prefix] + topics

        print(f"[rosbag] 录制话题: {len(topics)} 个")
        print(f"[rosbag] 保存到: {prefix}.bag")

        self._rosbag_process = subprocess.Popen(
            cmd,
            start_new_session=True,
        )

        return prefix

    def _start_controller(self):
        """启动控制器节点"""
        launch_file = self.ctrl_info['launch_file']
        launch_pkg = self.ctrl_info['launch_pkg']
        args = dict(self.ctrl_info.get('launch_args', {}))
        args['takeoff_height'] = str(self.takeoff_height)
        cmd = self._build_roslaunch_cmd(
            launch_pkg,
            launch_file,
            [f'{key}:={value}' for key, value in args.items()],
        )

        print(f"[controller] 启动: roslaunch {launch_pkg} {launch_file}")

        self._controller_process = subprocess.Popen(
            cmd,
            start_new_session=True,
        )
        self._started_launches.append({
            'role': 'controller',
            'package': launch_pkg,
            'launch_file': launch_file,
            'args': args,
            'resolved_path': self._resolve_launch_path(launch_pkg, launch_file),
        })

    def _start_planner(self):
        """启动规划器节点"""
        if self.planner_name == 'none':
            return

        launch_file = self.planner_info.get('launch_file')
        launch_pkg = self.planner_info.get('launch_pkg')
        if not launch_file or not launch_pkg:
            return

        args = dict(self.planner_info.get('args', {}))
        if self.task.waypoint_type:
            args.setdefault('waypoint_type', self.task.waypoint_type)

        cmd = self._build_roslaunch_cmd(
            launch_pkg,
            launch_file,
            [f'{key}:={value}' for key, value in args.items()],
        )

        print(f"[planner] 启动: roslaunch {launch_pkg} {launch_file}")

        self._planner_process = subprocess.Popen(
            cmd,
            start_new_session=True,
        )
        self._started_launches.append({
            'role': 'planner',
            'package': launch_pkg,
            'launch_file': launch_file,
            'args': args,
            'resolved_path': self._resolve_launch_path(launch_pkg, launch_file),
        })

    def _build_roslaunch_cmd(
        self,
        launch_pkg: Optional[str],
        launch_file: Optional[str],
        extra_args: Optional[List[str]] = None,
    ) -> List[str]:
        """构造 roslaunch 命令，优先使用可直接定位的 launch 绝对路径。"""
        if not launch_file:
            raise ValueError("launch_file 不能为空")

        cmd = ['roslaunch']
        resolved_launch = self._resolve_launch_path(launch_pkg, launch_file)
        if resolved_launch:
            cmd.append(resolved_launch)
        else:
            if not launch_pkg:
                raise ValueError(f"无法解析 launch 文件且未提供包名: {launch_file}")
            cmd.extend([launch_pkg, launch_file])

        if extra_args:
            cmd.extend(extra_args)
        return cmd

    def _resolve_launch_path(
        self,
        launch_pkg: Optional[str],
        launch_file: str,
    ) -> Optional[str]:
        """解析 launch 文件绝对路径，避免 roslaunch 对子目录查找失败。"""
        candidate_paths = []

        if os.path.isabs(launch_file):
            candidate_paths.append(launch_file)
        elif launch_pkg == 'opendrone':
            candidate_paths.append(os.path.join(_OPENDRONE_LAUNCH_ROOT, launch_file))

        for candidate in candidate_paths:
            if os.path.isfile(candidate):
                return candidate
        return None

    def _send_land_command(self):
        """发送降落指令"""
        try:
            subprocess.run(
                ['rosservice', 'call', '/land', 'true'],
                capture_output=True, timeout=10
            )
            print("[land] 已发送降落指令")
        except Exception as e:
            print(f"[land] 降落指令发送失败: {e}")

    @staticmethod
    def _signal_process_group(process, sig: int) -> bool:
        """向一个由 runner 创建的独立进程组发送信号。"""
        if not process:
            return False
        try:
            os.killpg(process.pid, sig)
            return True
        except ProcessLookupError:
            return False

    @staticmethod
    def _process_group_exists(process) -> bool:
        if not process:
            return False
        process.poll()
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    @classmethod
    def _wait_processes(cls, processes, timeout: float) -> List:
        """等待一批进程组退出；返回超时的进程组。"""
        deadline = time.monotonic() + timeout
        remaining = list(processes)
        while remaining and time.monotonic() < deadline:
            remaining = [
                item for item in remaining
                if cls._process_group_exists(item[0])
            ]
            if remaining:
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return [
            item for item in remaining
            if cls._process_group_exists(item[0])
        ]

    def _write_run_metadata(self, result: Dict) -> str:
        """保存与 bag 绑定的运行元数据，供独立 analyze 复现任务配置。"""
        path = os.path.join(self.record_dir, RUN_METADATA_FILENAME)
        try:
            os.makedirs(self.record_dir, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as metadata_file:
                json.dump(result, metadata_file, indent=2, ensure_ascii=False)
            return path
        except Exception as exc:
            print(f"[metadata] 保存运行元数据失败: {exc}")
            return ''

    def _build_run_result(
        self,
        bag_file: Optional[str],
        flight_duration: float,
        parameter_snapshot: Dict[str, object],
    ) -> Dict[str, object]:
        result = {
            'controller': self.controller_name,
            'controller_launch_args': dict(self.ctrl_info.get('launch_args', {})),
            'requested_controller_args': dict(self.controller_args),
            'task': self.task.name,
            'planner': self.planner_name,
            'planner_launch_args': dict(self.planner_info.get('args', {})),
            'requested_planner_args': dict(self.planner_args),
            'metric_profile': self.task.metric_profile,
            'bag_file': bag_file or '',
            'flight_duration': max(0.0, float(flight_duration)),
            'takeoff_height': self.takeoff_height,
            'task_duration': self.task.duration,
            'task_clock': self._task_clock_name,
            'emergency_occurred': self._emergency_occurred,
            'run_status': self._run_status_metadata(),
            'task_outcome': self._task_outcome_metadata(),
            'collision_summary': self._collision_metadata(),
            'process_cleanup': dict(self._cleanup_report),
            'lifecycle': self._lifecycle_metadata(),
            'topic_contract': {
                'controller': dict(self.controller_topics),
                'planner': dict(self.planner_topics),
                'task': {
                    'mission_path': MISSION_PATH_TOPIC,
                } if self.task.has_terminal_outcome else {},
                'collision': {
                    'contact_pulse': CONTACT_PULSE_TOPIC,
                },
                'recorded_topics': self._resolve_record_topics(),
            },
            'parameter_snapshot': parameter_snapshot,
            'algorithm_manifests': list(self._archived_manifest_files),
        }
        result.update(self.metadata_extra)
        return result

    def _find_bag_file(self, prefix: str) -> Optional[str]:
        """查找录制的 bag 文件"""
        import glob
        files = sorted(glob.glob(f'{prefix}*.bag'))
        return files[-1] if files else None

    def run(self) -> Optional[Dict]:
        """执行飞行评估。"""
        print("=" * 60)
        print(f"  飞行评估: {self.controller_name}")
        print(f"  指标集合: {self.task.metric_profile}")
        print(f"  任务: {self.task.description}")
        print(f"  规划器: {self.planner_name}")
        print("=" * 60)
        self._archive_manifests()

        prefix = ''
        start_time = 0.0
        parameter_snapshot = {}
        state_sub = None
        planner_sub = None
        mission_state_sub = None
        mission_path_sub = None
        odom_sub = None
        contact_sub = None
        try:
            self._transition(RunnerState.RECORDING, 'starting rosbag recording')
            prefix = self._start_rosbag()
            time.sleep(1)  # 等待 rosbag 启动

            self._start_controller()
            start_time = time.time()

            import rospy
            from nav_msgs.msg import Odometry, Path
            from opendrone.msg import MissionState, PlannerOutput
            from opendrone_gazebo_plugins.msg import ContactPulse
            from std_msgs.msg import Int8

            if not rospy.core.is_initialized():
                rospy.init_node('flight_eval', anonymous=True)
            self._use_ros_task_clock(rospy)
            state_sub = rospy.Subscriber(
                self.controller_topics['flight_state'],
                Int8,
                lambda msg: self._on_flight_state(msg.data),
            )
            contact_sub = rospy.Subscriber(
                CONTACT_PULSE_TOPIC,
                ContactPulse,
                self._on_contact_pulse,
                queue_size=1,
            )
            if self.planner_name != 'none':
                planner_sub = rospy.Subscriber(
                    self.planner_topics['planner_output'],
                    PlannerOutput,
                    lambda msg: self._on_planner_output(msg, rospy),
                    queue_size=1,
                )
                if 'mission_state' in self.planner_topics:
                    mission_state_sub = rospy.Subscriber(
                        self.planner_topics['mission_state'],
                        MissionState,
                        self._on_mission_state,
                        queue_size=10,
                    )
            if self.task.has_terminal_outcome:
                mission_path_sub = rospy.Subscriber(
                    MISSION_PATH_TOPIC,
                    Path,
                    self._on_mission_path,
                    queue_size=1,
                )
                odom_sub = rospy.Subscriber(
                    '/mavros/local_position/odom',
                    Odometry,
                    self._on_odometry,
                    queue_size=1,
                )

            self._transition(
                RunnerState.WAITING_FOR_CONTROLLER,
                'controller launched',
            )
            print(
                f"[wait] 等待控制器进入任务状态 "
                f"flight_state={self._target_flight_state}..."
            )
            print("[wait] (控制器会自动: 连接 → Offboard → 解锁 → 起飞)")
            controller_ready = self._wait_for_state(
                RunnerState.STARTING_PLANNER,
                timeout=60.0,
                rospy=rospy,
                label=f'waiting for flight_state={self._target_flight_state}',
            )
            if not controller_ready and self.runner_state == RunnerState.FAILED:
                print(
                    f"[error] 超时: 控制器未进入任务状态 "
                    f"flight_state={self._target_flight_state}"
                )
                return None

            if self.runner_state == RunnerState.STARTING_PLANNER:
                print(
                    f"[ok] 控制器已进入任务状态 "
                    f"flight_state={self._target_flight_state}"
                )
                self._start_planner()
                if self.planner_name == 'none':
                    self._transition(RunnerState.EXECUTING, 'task has no planner')
                else:
                    self._transition(
                        RunnerState.WAITING_FOR_PLANNER,
                        'planner launched',
                    )
                    with self._state_condition:
                        output_already_received = self._planner_output_received
                    if output_already_received:
                        self._transition(
                            RunnerState.EXECUTING,
                            'planner output arrived while planner was starting',
                        )

                    print("[wait] 等待首次 /planner/output 后开始任务计时...")
                    planner_ready = self._wait_for_state(
                        RunnerState.EXECUTING,
                        timeout=90.0,
                        rospy=rospy,
                        label='waiting for first planner output',
                    )
                    if not planner_ready and self.runner_state == RunnerState.FAILED:
                        print("[error] 超时: 规划器未发布 /planner/output")
                        return None
                    if planner_ready:
                        print("[ok] 已收到首次 /planner/output，开始任务计时。")

            if self.runner_state == RunnerState.EXECUTING:
                task_start = self._task_now()
                with self._state_condition:
                    self._task_evaluator.start(task_start)
                    if (
                        self._latest_mission_state is not None
                        and hasattr(self._task_evaluator, 'notify_mission_state')
                    ):
                        self._task_evaluator.notify_mission_state(
                            task_start, **self._latest_mission_state
                        )
                self._execution_end_reason = self._wait_for_task_end(rospy)
                if self._execution_end_reason == 'goal_reached':
                    print(
                        '[ok] 整体任务终点已到达'
                        f"（{self._task_evaluator.outcome.completion_time:.1f}s）"
                    )
                elif self._execution_end_reason == 'timeout':
                    print(
                        f"[warn] 任务在 {self.task.duration:.1f}s 期限内未完成"
                    )
                elif self._execution_end_reason == 'task_clock_stalled':
                    print(
                        '[warn] ROS 任务时钟停止推进，终止本次任务'
                    )
                elif not self._execution_end_reason and self.runner_state == RunnerState.ABORTING:
                    elapsed = self._task_now() - task_start
                    print(f"[warn] 任务中途终止 (已执行 {elapsed:.1f}s)")

            if self.runner_state == RunnerState.EXECUTING:
                end_reason = self._execution_end_reason or 'execution ended'
                if self.auto_land:
                    self._transition(RunnerState.REQUESTING_LAND, end_reason)
                    print("[land] 发送降落指令...")
                    self._send_land_command()
                    self._transition(RunnerState.WAITING_FOR_LANDED, 'land command sent')
                    landed = self._wait_for_state(
                        RunnerState.CAPTURING_ARTIFACTS,
                        timeout=30.0,
                        rospy=rospy,
                        label='waiting for LANDED',
                        timeout_state=RunnerState.CAPTURING_ARTIFACTS,
                    )
                    if landed:
                        print("[ok] 已着陆")
                    elif self.runner_state == RunnerState.CAPTURING_ARTIFACTS:
                        print("[warn] 等待着陆超时, 继续处理数据")
                else:
                    self._transition(
                        RunnerState.CAPTURING_ARTIFACTS,
                        f'{end_reason}; auto land disabled',
                    )

            if self.runner_state == RunnerState.ABORTING:
                print("[warn] 触发安全终止，跳过降落等待并保存运行数据。")
                self._transition(
                    RunnerState.CAPTURING_ARTIFACTS,
                    f'{self._execution_end_reason or "emergency"} data capture',
                )

            if self.runner_state == RunnerState.CAPTURING_ARTIFACTS:
                parameter_snapshot = self._capture_runtime_parameter_snapshot()
                time.sleep(2)  # 让 rosbag 收到最后一批状态和参数相关数据
                self._transition(RunnerState.COMPLETED, 'artifacts captured')

        except ImportError:
            self._task_evaluator.mark_unknown('run_failed')
            self._transition(RunnerState.FAILED, 'required ROS Python modules unavailable')
            print("[error] 缺少 rospy 或 opendrone 消息，无法运行 flight_eval。")
            return None
        except RunInterrupted as exc:
            self._task_evaluator.mark_unknown('run_interrupted')
            self._transition(RunnerState.INTERRUPTED, str(exc))
            print(f"[info] 运行已取消: {exc}")
            return None
        except KeyboardInterrupt:
            self._task_evaluator.mark_unknown('run_interrupted')
            self._transition(RunnerState.INTERRUPTED, 'KeyboardInterrupt')
            print("[info] 收到 Ctrl-C，停止所有 flight_eval 子进程...")
            raise
        except Exception:
            self._task_evaluator.mark_unknown('run_failed')
            self._transition(RunnerState.FAILED, 'unhandled runner exception')
            raise
        finally:
            for sub in (
                contact_sub, odom_sub, mission_path_sub, mission_state_sub,
                planner_sub, state_sub
            ):
                if sub is not None:
                    try:
                        sub.unregister()
                    except Exception:
                        pass
            self._cleanup()
            if self.runner_state in {RunnerState.FAILED, RunnerState.INTERRUPTED}:
                reason = (
                    'run_interrupted'
                    if self.runner_state == RunnerState.INTERRUPTED
                    else 'run_failed'
                )
                self._task_evaluator.mark_unknown(reason)
                partial_bag = self._find_bag_file(prefix) if prefix else None
                partial_duration = time.time() - start_time if start_time else 0.0
                partial_result = self._build_run_result(
                    partial_bag,
                    partial_duration,
                    parameter_snapshot,
                )
                self._write_run_metadata(partial_result)

        flight_duration = time.time() - start_time if start_time else 0.0

        bag_file = self._find_bag_file(prefix)
        if not bag_file:
            import glob
            files = sorted(glob.glob(os.path.join(self.record_dir, '*.bag')))
            if files:
                bag_file = files[-1]

        if not bag_file:
            print("[error] 未找到录制的 bag 文件")
            return None

        result = self._build_run_result(
            bag_file,
            flight_duration,
            parameter_snapshot,
        )
        result['run_metadata_file'] = self._write_run_metadata(result)
        print(f"[ok] Bag 文件: {bag_file}")
        return result

    def _cleanup(self):
        """一次性停止所有由 runner 创建的进程组。"""
        if self._cleaned_up:
            return
        processes = [
            (self._rosbag_process, 'rosbag'),
            (self._planner_process, 'planner'),
            (self._controller_process, 'controller'),
        ]
        self._cleanup_report = {
            'completed': False,
            'registered_process_groups': [
                name for process, name in processes if process is not None
            ],
            'remaining_process_groups': [],
        }
        active = [
            (process, name) for process, name in processes
            if self._process_group_exists(process)
        ]
        if not active:
            self._cleaned_up = True
            self._cleanup_report['completed'] = True
            return

        for process, name in active:
            print(f"[{name}] 正在停止...")
            self._signal_process_group(process, signal.SIGINT)
        active = self._wait_processes(active, timeout=8.0)

        for process, _ in active:
            self._signal_process_group(process, signal.SIGTERM)
        active = self._wait_processes(active, timeout=3.0)

        for process, _ in active:
            self._signal_process_group(process, signal.SIGKILL)
        active = self._wait_processes(active, timeout=2.0)
        self._cleaned_up = not active
        self._cleanup_report['completed'] = self._cleaned_up
        self._cleanup_report['remaining_process_groups'] = [
            name for _, name in active
        ]
        if active:
            names = ', '.join(name for _, name in active)
            print(f'[cleanup] 以下进程组在 SIGKILL 后仍可见: {names}')
        else:
            print('[cleanup] 所有 flight_eval 子进程已退出')

    def __del__(self):
        self._cleanup()
