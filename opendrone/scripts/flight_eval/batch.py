#!/usr/bin/env python3
"""YAML-driven flight evaluation batches."""

from __future__ import annotations

import copy
import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - only reached on incomplete installations
    yaml = None

from .controllers import CONTROLLER_REGISTRY, get_controller_launch
from .manifests import ManifestError, load_manifest_files
from .planners import PLANNER_REGISTRY, get_planner_launch
from .runner import FlightRunner
from .tasks import TASK_REGISTRY


_PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
_OPENDRONE_ROOT = os.path.dirname(_PACKAGE_ROOT)
_WORKSPACE_ROOT = os.path.dirname(os.path.dirname(_OPENDRONE_ROOT))
_SITL_CONFIG_ROOT = os.path.join(_PACKAGE_ROOT, 'sitl_config')

_CASE_NAME_PATTERN = re.compile(r'[^A-Za-z0-9._-]+')
_RUN_OPTION_KEYS = {
    'task', 'takeoff_height', 'duration',
    'auto_land', 'extra_topics', 'max_collision_episodes',
    'collision_episode_gap',
}
_ROOT_KEYS = {
    'version', 'name', 'manifests', 'defaults', 'experiments',
    'output_dir', 'analysis',
}
_DEFAULT_KEYS = {'simulation', 'run'}
_EXPERIMENT_KEYS = {
    'name', 'repeat', 'controllers', 'controller', 'controller_args',
    'planners', 'planner', 'planner_args', 'simulation', 'run',
} | _RUN_OPTION_KEYS
_GAZEBO_SHORTCUT_KEYS = {
    'world', 'sensor', 'sdf', 'vehicle', 'x', 'y', 'z', 'R', 'P', 'Y',
    'gui', 'debug', 'verbose', 'paused', 'respawn_gazebo', 'model_paths',
}
_SIMULATION_KEYS = {'gazebo', 'px4', 'mavros', 'timeouts', 'transport'} | _GAZEBO_SHORTCUT_KEYS
_GAZEBO_KEYS = {
    'world', 'sensor', 'sdf', 'vehicle', 'x', 'y', 'z', 'R', 'P', 'Y',
    'gui', 'debug', 'verbose', 'paused', 'respawn_gazebo', 'model_paths',
}
_PX4_KEYS = {'source_dir', 'build_dir', 'sim_model', 'env', 'startup_script'}
_MAVROS_KEYS = {
    'fcu_url', 'gcs_url', 'tgt_system', 'tgt_component', 'log_output',
    'fcu_protocol', 'respawn_mavros', 'config_yaml',
}
_TIMEOUT_KEYS = {'gazebo', 'mavros', 'px4', 'px4_ready', 'reset'}
_TRANSPORT_KEYS = {'gazebo_px4', 'port', 'environment_dir'}


class BatchConfigError(ValueError):
    """YAML 的结构或值无法构成可运行的批次实验。"""


class _BatchTerminationGuard:
    """把 SIGINT/SIGTERM 变成一次可清理的批次取消。"""

    def __init__(self):
        self._original_handlers: Dict[int, Any] = {}
        self._installed = False

    def __enter__(self):
        if threading.current_thread() is not threading.main_thread():
            return self
        for signum in (signal.SIGINT, signal.SIGTERM):
            self._original_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, self._handle)
        self._installed = True
        return self

    def _handle(self, signum, frame) -> None:
        del signum, frame
        for guarded_signal in self._original_handlers:
            signal.signal(guarded_signal, signal.SIG_IGN)
        raise KeyboardInterrupt

    def __exit__(self, exc_type, exc_value, traceback_value):
        del exc_type, exc_value, traceback_value
        if self._installed:
            for signum, handler in self._original_handlers.items():
                signal.signal(signum, handler)


def _as_mapping(value: Any, label: str) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise BatchConfigError(f'{label} 必须是 YAML mapping/object')
    return dict(value)


def _as_list(value: Any, label: str) -> List[Any]:
    if isinstance(value, list):
        return list(value)
    return [value]


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> Dict[str, Any]:
    """递归合并 mapping；列表和标量由后者完整替换。"""
    merged = copy.deepcopy(dict(base))
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _slug(value: str) -> str:
    text = _CASE_NAME_PATTERN.sub('_', str(value)).strip('._-')
    return text or 'case'


def _ros_arg(value: Any) -> str:
    """ROS launch 参数的确定性文本表示。"""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return ''
    return str(value)


def _ensure_positive_number(value: Any, label: str, default: float) -> float:
    if value is None:
        return default
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise BatchConfigError(f'{label} 必须是正数，得到: {value!r}') from exc
    if number <= 0:
        raise BatchConfigError(f'{label} 必须大于 0，得到: {value!r}')
    return number


def _require_true(value: Any, label: str) -> bool:
    """批次重启 PX4 时，外部环境必须具备自动重启能力。"""
    if value is None:
        return True
    if value is not True:
        raise BatchConfigError(f'{label} 必须为 true；flight_eval 依赖它在 PX4 重启后自动刷新。')
    return True


def _resolve_config_path(path_value: str, config_dir: str) -> str:
    path = os.path.expanduser(str(path_value))
    if not os.path.isabs(path):
        path = os.path.join(config_dir, path)
    return os.path.abspath(path)


def _normalise_simulation(raw: Mapping[str, Any], label: str) -> Dict[str, Any]:
    """支持 simulation.gazebo.*，也支持 world/sensor 这类常用简写。"""
    simulation = copy.deepcopy(dict(raw))
    unknown_simulation = set(simulation) - _SIMULATION_KEYS
    if unknown_simulation:
        raise BatchConfigError(
            f'{label} 包含不支持的字段: {sorted(unknown_simulation)}'
        )
    gazebo = _as_mapping(simulation.get('gazebo'), f'{label}.gazebo')
    for key in _GAZEBO_SHORTCUT_KEYS:
        if key in simulation:
            if key in gazebo:
                raise BatchConfigError(
                    f'{label} 同时设置了 simulation.{key} 与 simulation.gazebo.{key}'
                )
            gazebo[key] = simulation.pop(key)
    simulation['gazebo'] = gazebo
    simulation['px4'] = _as_mapping(simulation.get('px4'), f'{label}.px4')
    simulation['mavros'] = _as_mapping(simulation.get('mavros'), f'{label}.mavros')
    simulation['timeouts'] = _as_mapping(simulation.get('timeouts'), f'{label}.timeouts')
    simulation['transport'] = _as_mapping(simulation.get('transport'), f'{label}.transport')
    for field_name, value, allowed in (
        ('gazebo', simulation['gazebo'], _GAZEBO_KEYS),
        ('px4', simulation['px4'], _PX4_KEYS),
        ('mavros', simulation['mavros'], _MAVROS_KEYS),
        ('timeouts', simulation['timeouts'], _TIMEOUT_KEYS),
        ('transport', simulation['transport'], _TRANSPORT_KEYS),
    ):
        unknown = set(value) - allowed
        if unknown:
            raise BatchConfigError(
                f'{label}.{field_name} 包含不支持的字段: {sorted(unknown)}'
            )
    _require_true(gazebo.get('respawn_gazebo'), f'{label}.gazebo.respawn_gazebo')
    _require_true(simulation['mavros'].get('respawn_mavros'), f'{label}.mavros.respawn_mavros')
    return simulation


def _normalise_algorithm_item(
    value: Any,
    kind: str,
    common_args: Mapping[str, Any],
) -> Dict[str, Any]:
    if isinstance(value, str):
        item = {'name': value}
    elif isinstance(value, Mapping):
        item = dict(value)
    else:
        raise BatchConfigError(f'{kind} 必须是名称字符串或带 name 的 mapping，得到: {value!r}')

    name = item.get('name')
    if not isinstance(name, str) or not name:
        raise BatchConfigError(f'{kind}.name 必须是非空字符串')

    args = _deep_merge(common_args, _as_mapping(item.get('args'), f'{kind}({name}).args'))
    normalised_args = {}
    for key, arg_value in args.items():
        if not isinstance(key, str) or not key:
            raise BatchConfigError(f'{kind}({name}).args 的 key 必须是非空字符串')
        normalised_args[key] = _ros_arg(arg_value)
    unknown = set(item) - {'name', 'args'}
    if unknown:
        raise BatchConfigError(f'{kind}({name}) 包含未知字段: {sorted(unknown)}')
    return {'name': name, 'args': normalised_args}


def _normalise_algorithm_items(
    experiment: Mapping[str, Any],
    plural_key: str,
    singular_key: str,
    registry: Mapping[str, Any],
    common_args_key: str,
    default: Optional[Any] = None,
) -> List[Dict[str, Any]]:
    if plural_key in experiment and singular_key in experiment:
        raise BatchConfigError(
            f'experiment 不能同时使用 {plural_key} 和 {singular_key}'
        )
    if plural_key in experiment:
        raw_items = _as_list(experiment[plural_key], plural_key)
    elif singular_key in experiment:
        raw_items = [experiment[singular_key]]
    elif default is not None:
        raw_items = [default]
    else:
        raise BatchConfigError(f'experiment 缺少 {plural_key} 或 {singular_key}')

    common_args = _as_mapping(experiment.get(common_args_key), common_args_key)
    kind = 'planner' if singular_key == 'planner' else 'controller'
    items = [
        _normalise_algorithm_item(item, kind, common_args)
        for item in raw_items
    ]
    for item in items:
        name = item['name']
        if name not in registry:
            raise BatchConfigError(
                f'未知 {kind}: {name}. 可用: {sorted(registry.keys())}'
            )
        if kind == 'planner':
            get_planner_launch(name, item['args'])
        else:
            get_controller_launch(name, item['args'])
    return items


@dataclass
class BatchCase:
    """YAML matrix 展开后的一条独立 flight_eval 样本。"""

    index: int
    experiment_name: str
    repetition: int
    task: str
    controller: Dict[str, Any]
    planner: Dict[str, Any]
    run_options: Dict[str, Any]
    simulation: Dict[str, Any]

    @property
    def case_name(self) -> str:
        return (
            f'{self.index:03d}_{_slug(self.experiment_name)}_'
            f'{_slug(self.controller["name"])}_{_slug(self.planner["name"])}_'
            f'r{self.repetition:02d}'
        )

    @property
    def simulation_signature(self) -> str:
        return json.dumps(self.simulation, sort_keys=True, ensure_ascii=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'index': self.index,
            'case_name': self.case_name,
            'experiment_name': self.experiment_name,
            'repetition': self.repetition,
            'task': self.task,
            'controller': copy.deepcopy(self.controller),
            'planner': copy.deepcopy(self.planner),
            'metric_profile': TASK_REGISTRY[self.task]().metric_profile,
            'run_options': copy.deepcopy(self.run_options),
            'simulation': copy.deepcopy(self.simulation),
        }


@dataclass
class BatchDefinition:
    name: str
    config_path: str
    output_dir: str
    analysis_enabled: bool
    cases: List[BatchCase]
    manifest_paths: List[str]


def load_batch_definition(config_path: str, output_dir: Optional[str] = None) -> BatchDefinition:
    """读取、验证 YAML，并把 controller/planner/repeat 展开成实验样本。"""
    if yaml is None:
        raise RuntimeError('批次 YAML 需要 PyYAML，请安装: pip install PyYAML')

    config_path = os.path.abspath(os.path.expanduser(config_path))
    if not os.path.isfile(config_path):
        raise BatchConfigError(f'找不到批次配置: {config_path}')
    config_dir = os.path.dirname(config_path)
    try:
        with open(config_path, 'r', encoding='utf-8') as stream:
            root = yaml.safe_load(stream) or {}
    except yaml.YAMLError as exc:
        raise BatchConfigError(f'YAML 解析失败: {exc}') from exc
    root = _as_mapping(root, 'batch YAML 根对象')
    unknown_root = set(root) - _ROOT_KEYS
    if unknown_root:
        raise BatchConfigError(
            f'batch YAML 包含不支持的字段: {sorted(unknown_root)}'
        )

    version = root.get('version', 1)
    if version != 1:
        raise BatchConfigError(f'暂只支持 version: 1，得到: {version!r}')
    batch_name = root.get('name')
    if not isinstance(batch_name, str) or not batch_name.strip():
        raise BatchConfigError('name 必须是非空字符串')

    raw_manifest_paths = root.get('manifests', [])
    if not isinstance(raw_manifest_paths, list) or not all(
        isinstance(path, str) and path for path in raw_manifest_paths
    ):
        raise BatchConfigError('manifests 必须是路径字符串 list')
    manifest_paths = [
        _resolve_config_path(path, config_dir) for path in raw_manifest_paths
    ]
    try:
        load_manifest_files(
            manifest_paths, CONTROLLER_REGISTRY, PLANNER_REGISTRY
        )
    except ManifestError as exc:
        raise BatchConfigError(str(exc)) from exc

    defaults = _as_mapping(root.get('defaults'), 'defaults')
    unknown_defaults = set(defaults) - _DEFAULT_KEYS
    if unknown_defaults:
        raise BatchConfigError(
            f'defaults 包含不支持的字段: {sorted(unknown_defaults)}'
        )
    default_simulation = _normalise_simulation(
        _as_mapping(defaults.get('simulation'), 'defaults.simulation'),
        'defaults.simulation',
    )
    default_run = _as_mapping(defaults.get('run'), 'defaults.run')
    unknown_default_run = set(default_run) - _RUN_OPTION_KEYS
    if unknown_default_run:
        raise BatchConfigError(
            f'defaults.run 包含不支持的字段: {sorted(unknown_default_run)}'
        )

    raw_experiments = root.get('experiments')
    if not isinstance(raw_experiments, list) or not raw_experiments:
        raise BatchConfigError('experiments 必须是非空 YAML list')

    cases: List[BatchCase] = []
    for experiment_index, raw_experiment in enumerate(raw_experiments, start=1):
        experiment = _as_mapping(raw_experiment, f'experiments[{experiment_index - 1}]')
        unknown_experiment = set(experiment) - _EXPERIMENT_KEYS
        if unknown_experiment:
            raise BatchConfigError(
                f'experiments[{experiment_index - 1}] 包含不支持的字段: '
                f'{sorted(unknown_experiment)}'
            )
        experiment_name = experiment.get('name', f'experiment_{experiment_index}')
        if not isinstance(experiment_name, str) or not experiment_name.strip():
            raise BatchConfigError(f'experiments[{experiment_index - 1}].name 必须是非空字符串')

        experiment_run = _deep_merge(
            default_run,
            _as_mapping(experiment.get('run'), f'experiments[{experiment_index - 1}].run'),
        )
        for key in _RUN_OPTION_KEYS:
            if key in experiment:
                experiment_run[key] = copy.deepcopy(experiment[key])
        unknown_experiment_run = set(experiment_run) - _RUN_OPTION_KEYS
        if unknown_experiment_run:
            raise BatchConfigError(
                f'experiment {experiment_name}.run 包含不支持的字段: '
                f'{sorted(unknown_experiment_run)}'
            )

        task = experiment_run.get('task')
        if not isinstance(task, str) or task not in TASK_REGISTRY:
            raise BatchConfigError(
                f'experiment {experiment_name} 的 task 无效: {task!r}. '
                f'可用: {sorted(TASK_REGISTRY.keys())}'
            )
        if 'auto_land' in experiment_run and not isinstance(experiment_run['auto_land'], bool):
            raise BatchConfigError(f'experiment {experiment_name}.auto_land 必须是 true 或 false')
        extra_topics = experiment_run.get('extra_topics', [])
        if not isinstance(extra_topics, list) or not all(isinstance(topic, str) for topic in extra_topics):
            raise BatchConfigError(
                f'experiment {experiment_name}.extra_topics 必须是字符串 list'
            )
        for duration_key in ('takeoff_height', 'duration'):
            if duration_key in experiment_run:
                _ensure_positive_number(
                    experiment_run[duration_key],
                    f'experiment {experiment_name}.{duration_key}',
                    1.0,
                )
        if 'max_collision_episodes' in experiment_run:
            collision_limit = experiment_run['max_collision_episodes']
            if (
                not isinstance(collision_limit, int)
                or isinstance(collision_limit, bool)
                or collision_limit < 1
            ):
                raise BatchConfigError(
                    f'experiment {experiment_name}.max_collision_episodes '
                    '必须是正整数'
                )
        if 'collision_episode_gap' in experiment_run:
            _ensure_positive_number(
                experiment_run['collision_episode_gap'],
                f'experiment {experiment_name}.collision_episode_gap',
                0.5,
            )

        repeat = experiment.get('repeat', 1)
        if not isinstance(repeat, int) or isinstance(repeat, bool) or repeat < 1:
            raise BatchConfigError(f'experiment {experiment_name}.repeat 必须是正整数')

        controllers = _normalise_algorithm_items(
            experiment, 'controllers', 'controller', CONTROLLER_REGISTRY,
            'controller_args', default=None,
        )
        planners = _normalise_algorithm_items(
            experiment, 'planners', 'planner', PLANNER_REGISTRY,
            'planner_args', default='none',
        )

        experiment_simulation = _normalise_simulation(
            _as_mapping(experiment.get('simulation'),
                        f'experiments[{experiment_index - 1}].simulation'),
            f'experiments[{experiment_index - 1}].simulation',
        )
        simulation = _deep_merge(default_simulation, experiment_simulation)

        for controller in controllers:
            for planner in planners:
                controller_info = get_controller_launch(
                    controller['name'], controller['args']
                )
                contract_planner_name = planner['name']
                if contract_planner_name == 'none':
                    contract_planner_name = (
                        TASK_REGISTRY[task]().planner_name_override or contract_planner_name
                    )
                planner_info = get_planner_launch(
                    contract_planner_name, planner['args']
                )
                for kind, algorithm_name, info in (
                    ('controller', controller['name'], controller_info),
                    ('planner', contract_planner_name, planner_info),
                ):
                    allowed_tasks = info.get('tasks', [])
                    if allowed_tasks and task not in allowed_tasks:
                        raise BatchConfigError(
                            f'{kind} {algorithm_name} 不允许用于 task={task}. '
                            f'可用: {allowed_tasks}'
                        )
                for repetition in range(1, repeat + 1):
                    run_options = copy.deepcopy(experiment_run)
                    cases.append(BatchCase(
                        index=len(cases) + 1,
                        experiment_name=experiment_name,
                        repetition=repetition,
                        task=task,
                        controller=copy.deepcopy(controller),
                        planner=copy.deepcopy(planner),
                        run_options=run_options,
                        simulation=copy.deepcopy(simulation),
                    ))

    configured_output = output_dir if output_dir is not None else root.get('output_dir')
    if configured_output:
        resolved_output_dir = _resolve_config_path(str(configured_output), config_dir)
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        resolved_output_dir = os.path.join(
            _WORKSPACE_ROOT, 'eval_runs', 'batches',
            f'{_slug(batch_name)}_{timestamp}',
        )

    analysis_enabled = root.get('analysis', True)
    if not isinstance(analysis_enabled, bool):
        raise BatchConfigError('analysis 必须是 true 或 false')

    for case in cases:
        source_dir = _as_mapping(
            case.simulation.get('px4'), 'simulation.px4'
        ).get('source_dir')
        if not source_dir:
            raise BatchConfigError(
                'simulation.px4.source_dir 为必填项；请填写 PX4-Autopilot 源码目录。'
            )
        resolved_source_dir = _resolve_config_path(str(source_dir), config_dir)
        if not os.path.isdir(resolved_source_dir):
            raise BatchConfigError(
                f'simulation.px4.source_dir 不存在: {resolved_source_dir}'
            )

    external_environments = {
        json.dumps(
            {
                'gazebo': case.simulation.get('gazebo', {}),
                'mavros': case.simulation.get('mavros', {}),
                'transport': case.simulation.get('transport', {}),
            },
            sort_keys=True,
            ensure_ascii=False,
        )
        for case in cases
    }
    if len(external_environments) != 1:
        raise BatchConfigError(
            '同一 batch 只能使用一套 gazebo/mavros 配置；请按 world/sensor 拆分 YAML，'
            '并由使用者分别启动对应的外部环境。'
        )

    return BatchDefinition(
        name=batch_name,
        config_path=config_path,
        output_dir=resolved_output_dir,
        analysis_enabled=analysis_enabled,
        cases=cases,
        manifest_paths=manifest_paths,
    )


def format_batch_plan(definition: BatchDefinition) -> str:
    """生成 dry-run 与日志共用的简洁展开结果。"""
    lines = [
        f'批次: {definition.name}',
        f'配置: {definition.config_path}',
        f'输出: {definition.output_dir}',
        f'样本数: {len(definition.cases)}',
    ]
    for case in definition.cases:
        gazebo = case.simulation.get('gazebo', {})
        lines.append(
            f'  {case.index:03d}. {case.case_name}: task={case.task}, '
            f'controller={case.controller["name"]}, planner={case.planner["name"]}, '
            f'world={gazebo.get("world", "empty")}, '
            f'sensor={gazebo.get("sensor", "none")} — 使用外部 Gazebo + MAVROS，重启 PX4'
        )
    return '\n'.join(lines)


@dataclass
class _ManagedProcess:
    name: str
    process: subprocess.Popen
    log_path: str

    def ensure_running(self) -> None:
        exit_code = self.process.poll()
        if exit_code is not None:
            raise RuntimeError(
                f'{self.name} 已提前退出 (exit={exit_code})，请查看日志: {self.log_path}'
            )

    def stop(self) -> bool:
        for sig, timeout in (
            (signal.SIGINT, 10.0),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 3.0),
        ):
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.process.poll()
                try:
                    os.killpg(self.process.pid, 0)
                except ProcessLookupError:
                    return True
                time.sleep(0.05)
        try:
            self.process.wait(timeout=0)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(self.process.pid, 0)
        except ProcessLookupError:
            return True
        return False


class _MavrosStateObserver:
    """Process-owned subscription used for MAVROS link lifecycle checks."""

    def __init__(self, rospy_module, state_type):
        self._lock = threading.Lock()
        self._connected: Optional[bool] = None
        self._sequence = 0
        self._received_at = 0.0
        self._subscriber = rospy_module.Subscriber(
            '/mavros/state', state_type, self._on_state, queue_size=1,
        )

    def _on_state(self, message) -> None:
        with self._lock:
            self._connected = bool(message.connected)
            self._sequence += 1
            self._received_at = time.monotonic()

    @property
    def sequence(self) -> int:
        with self._lock:
            return self._sequence

    def matches(
        self,
        connected: bool,
        after_sequence: int,
        max_age: float,
    ) -> bool:
        with self._lock:
            return (
                self._sequence > after_sequence
                and self._connected is connected
                and time.monotonic() - self._received_at <= max_age
            )

    def close(self) -> None:
        subscriber, self._subscriber = self._subscriber, None
        if subscriber is not None:
            try:
                subscriber.unregister()
            except Exception:
                pass


class _SimulationSession:
    """连接使用者的 Gazebo/MAVROS，并按 case 生命周期运行 PX4。"""

    def __init__(
        self,
        simulation: Mapping[str, Any],
        config_dir: str,
        log_dir: str,
    ):
        self.simulation = copy.deepcopy(dict(simulation))
        self.config_dir = config_dir
        self.log_dir = log_dir
        self.px4_process: Optional[_ManagedProcess] = None
        self._mavros_state: Optional[_MavrosStateObserver] = None
        self._mavros_transition_sequence = 0
        self._has_run_case = False

        self.px4_source_dir = ''
        self.px4_build_dir = ''
        self.px4_sim_model = ''
        self.world_path = ''
        self.sdf_path = ''
        self.mavros_config_path = ''
        self.environment: Dict[str, str] = {}
        self.gazebo_args: Dict[str, Any] = {}
        self.mavros_args: Dict[str, Any] = {}
        self.timeouts: Dict[str, float] = {}
        self.transport: Dict[str, Any] = {}

    @staticmethod
    def _prepend_env_path(environment: Dict[str, str], variable: str, value: str) -> None:
        existing = environment.get(variable, '')
        environment[variable] = value if not existing else f'{value}{os.pathsep}{existing}'

    def _discover_px4_source(self, requested: Optional[str]) -> str:
        if not requested:
            raise BatchConfigError(
                'simulation.px4.source_dir 为必填项；请填写 PX4-Autopilot 源码目录。'
            )
        candidate = _resolve_config_path(str(requested), self.config_dir)
        if os.path.isdir(candidate):
            return candidate
        raise BatchConfigError(f'px4.source_dir 不存在: {candidate}')

    def _resolve_world(self, world: Any) -> str:
        if not world:
            world = 'empty'
        if not isinstance(world, str):
            raise BatchConfigError(f'gazebo.world 必须是字符串，得到: {world!r}')

        px4_world_dir = os.path.join(
            self.px4_source_dir, 'Tools', 'simulation', 'gazebo-classic',
            'sitl_gazebo-classic', 'worlds'
        )
        local_world_dir = os.path.join(_SITL_CONFIG_ROOT, 'worlds')
        aliases = {
            'empty': os.path.join(px4_world_dir, 'empty.world'),
        }
        if world in aliases:
            candidate = aliases[world]
        elif not os.path.isabs(world) and not world.endswith('.world'):
            candidate = os.path.join(local_world_dir, f'{world}.world')
        else:
            candidate = _resolve_config_path(world, self.config_dir)
            if not os.path.isfile(candidate) and not os.path.isabs(world):
                candidate = os.path.join(local_world_dir, world)
        if not os.path.isfile(candidate):
            raise BatchConfigError(
                f'找不到 Gazebo world: {world!r}（解析为 {candidate}）'
            )
        return os.path.abspath(candidate)

    def _resolve_sdf(self, gazebo: Mapping[str, Any]) -> str:
        sdf = gazebo.get('sdf')
        if sdf:
            candidate = _resolve_config_path(str(sdf), self.config_dir)
        else:
            sensor = str(gazebo.get('sensor', 'none')).lower()
            px4_models_dir = os.path.join(
                self.px4_source_dir, 'Tools', 'simulation', 'gazebo-classic',
                'sitl_gazebo-classic', 'models'
            )
            local_models_dir = os.path.join(_SITL_CONFIG_ROOT, 'models')
            sensor_models = {
                'none': ('iris', 'iris.sdf'),
                'iris': ('iris', 'iris.sdf'),
                'default': ('iris', 'iris.sdf'),
                'mid360': ('iris_mid360', 'iris_mid360.sdf'),
                'depth_camera': ('iris_depth_camera_new', 'iris_depth_camera_new.sdf'),
            }
            if sensor not in sensor_models:
                raise BatchConfigError(
                    'gazebo.sensor 仅支持 none / mid360 / depth_camera；'
                    '自定义模型请设置 gazebo.sdf'
                )
            directory, filename = sensor_models[sensor]
            local_candidate = os.path.join(local_models_dir, directory, filename)
            candidate = (
                local_candidate if sensor not in {'none', 'iris', 'default'}
                and os.path.isfile(local_candidate)
                else os.path.join(px4_models_dir, directory, filename)
            )
        if not os.path.isfile(candidate):
            raise BatchConfigError(f'找不到 Gazebo SDF: {candidate}')
        return os.path.abspath(candidate)

    def _prepare(self) -> None:
        px4 = _as_mapping(self.simulation.get('px4'), 'simulation.px4')
        gazebo = _as_mapping(self.simulation.get('gazebo'), 'simulation.gazebo')
        mavros = _as_mapping(self.simulation.get('mavros'), 'simulation.mavros')
        timeouts = _as_mapping(self.simulation.get('timeouts'), 'simulation.timeouts')
        transport = _as_mapping(self.simulation.get('transport'), 'simulation.transport')

        self.px4_source_dir = self._discover_px4_source(px4.get('source_dir'))
        requested_build = px4.get('build_dir')
        self.px4_build_dir = (
            _resolve_config_path(str(requested_build), self.config_dir)
            if requested_build
            else os.path.join(self.px4_source_dir, 'build', 'px4_sitl_default')
        )
        self.px4_build_dir = os.path.abspath(self.px4_build_dir)
        px4_binary = os.path.join(self.px4_build_dir, 'bin', 'px4')
        px4_etc = os.path.join(self.px4_build_dir, 'etc')
        if not os.path.isfile(px4_binary) or not os.path.isdir(px4_etc):
            raise BatchConfigError(
                f'PX4 SITL build 不完整: {self.px4_build_dir}（需要 bin/px4 和 etc/）'
            )
        self.px4_sim_model = str(px4.get('sim_model', 'gazebo-classic_iris'))
        self.world_path = self._resolve_world(gazebo.get('world', 'empty'))
        self.sdf_path = self._resolve_sdf(gazebo)

        transport_mode = str(transport.get('gazebo_px4', 'tcp')).lower()
        if transport_mode not in {'tcp', 'udp'}:
            raise BatchConfigError('transport.gazebo_px4 仅支持 tcp 或 udp')
        transport_port = int(_ensure_positive_number(
            transport.get('port'), 'transport.port', 14560.0 if transport_mode == 'udp' else 4560.0,
        ))
        if transport_port > 65535:
            raise BatchConfigError('transport.port 必须小于等于 65535')
        environment_dir = transport.get('environment_dir')
        self.transport = {
            'gazebo_px4': transport_mode,
            'port': transport_port,
            'environment_dir': (
                _resolve_config_path(str(environment_dir), self.config_dir)
                if environment_dir else ''
            ),
            'startup_script': (
                _resolve_config_path(str(px4['startup_script']), self.config_dir)
                if px4.get('startup_script') else ''
            ),
        }
        if self.transport['startup_script'] and not os.path.isfile(self.transport['startup_script']):
            raise BatchConfigError(
                f'px4.startup_script 不存在: {self.transport["startup_script"]}'
            )

        config_yaml = mavros.get('config_yaml')
        self.mavros_config_path = (
            _resolve_config_path(str(config_yaml), self.config_dir)
            if config_yaml else os.path.join(_SITL_CONFIG_ROOT, 'launch', 'px4_config.yaml')
        )
        if not os.path.isfile(self.mavros_config_path):
            raise BatchConfigError(f'找不到 MAVROS config_yaml: {self.mavros_config_path}')

        environment = dict(os.environ)
        self._prepend_env_path(environment, 'ROS_PACKAGE_PATH', _OPENDRONE_ROOT)
        self._prepend_env_path(environment, 'ROS_PACKAGE_PATH', self.px4_source_dir)
        self._prepend_env_path(
            environment, 'ROS_PACKAGE_PATH',
            os.path.join(
                self.px4_source_dir, 'Tools', 'simulation', 'gazebo-classic',
                'sitl_gazebo-classic'
            ),
        )
        self._prepend_env_path(
            environment, 'GAZEBO_PLUGIN_PATH',
            os.path.join(self.px4_build_dir, 'build_gazebo-classic'),
        )
        self._prepend_env_path(
            environment, 'GAZEBO_MODEL_PATH',
            os.path.join(_SITL_CONFIG_ROOT, 'models'),
        )
        self._prepend_env_path(
            environment, 'GAZEBO_MODEL_PATH',
            os.path.join(
                self.px4_source_dir, 'Tools', 'simulation', 'gazebo-classic',
                'sitl_gazebo-classic', 'models'
            ),
        )
        self.gazebo_model_paths = []
        for raw_model_path in _as_list(gazebo.get('model_paths', []), 'gazebo.model_paths'):
            model_path = _resolve_config_path(str(raw_model_path), self.config_dir)
            if not os.path.isdir(model_path):
                raise BatchConfigError(f'gazebo.model_paths 中目录不存在: {model_path}')
            self.gazebo_model_paths.append(model_path)
            self._prepend_env_path(environment, 'GAZEBO_MODEL_PATH', model_path)
        self._prepend_env_path(
            environment, 'LD_LIBRARY_PATH',
            os.path.join(self.px4_build_dir, 'build_gazebo-classic'),
        )
        for key, value in _as_mapping(px4.get('env'), 'simulation.px4.env').items():
            if not isinstance(key, str) or not key:
                raise BatchConfigError('simulation.px4.env 的 key 必须是非空字符串')
            environment[key] = _ros_arg(value)
        environment['PX4_SIM_MODEL'] = self.px4_sim_model
        self.environment = environment

        self.gazebo_args = {
            'world': self.world_path,
            'sdf': self.sdf_path,
            'vehicle': gazebo.get('vehicle', 'iris'),
            'x': gazebo.get('x', 0), 'y': gazebo.get('y', 0), 'z': gazebo.get('z', 0),
            'R': gazebo.get('R', 0), 'P': gazebo.get('P', 0), 'Y': gazebo.get('Y', 0),
            'gui': gazebo.get('gui', True),
            'debug': gazebo.get('debug', False),
            'verbose': gazebo.get('verbose', False),
            'paused': gazebo.get('paused', False),
            'respawn_gazebo': _require_true(
                gazebo.get('respawn_gazebo'), 'gazebo.respawn_gazebo',
            ),
        }
        if not isinstance(self.gazebo_args['vehicle'], str) or not self.gazebo_args['vehicle']:
            raise BatchConfigError('gazebo.vehicle 必须是非空字符串')
        self.mavros_args = {
            'fcu_url': mavros.get('fcu_url', 'udp://:14540@localhost:14557'),
            'gcs_url': mavros.get('gcs_url', ''),
            'tgt_system': mavros.get('tgt_system', 1),
            'tgt_component': mavros.get('tgt_component', 1),
            'log_output': mavros.get('log_output', 'screen'),
            'fcu_protocol': mavros.get('fcu_protocol', 'v2.0'),
            'respawn_mavros': _require_true(
                mavros.get('respawn_mavros'), 'mavros.respawn_mavros',
            ),
            'config_yaml': self.mavros_config_path,
        }
        self.timeouts = {
            'gazebo': _ensure_positive_number(timeouts.get('gazebo'), 'timeouts.gazebo', 60.0),
            'mavros': _ensure_positive_number(timeouts.get('mavros'), 'timeouts.mavros', 30.0),
            'px4': _ensure_positive_number(timeouts.get('px4'), 'timeouts.px4', 60.0),
            'px4_ready': _ensure_positive_number(
                timeouts.get('px4_ready'), 'timeouts.px4_ready', 3.0,
            ),
            'reset': _ensure_positive_number(timeouts.get('reset'), 'timeouts.reset', 10.0),
        }

    def _verify_udp_environment(self) -> None:
        environment_dir = self.transport['environment_dir']
        if not environment_dir:
            raise BatchConfigError(
                'UDP 模式需要 transport.environment_dir；请先运行 prepare-environment。'
            )
        manifest_path = os.path.join(environment_dir, 'environment.json')
        try:
            with open(manifest_path, 'r', encoding='utf-8') as stream:
                manifest = json.load(stream)
        except (OSError, ValueError) as exc:
            raise BatchConfigError(
                f'UDP 环境清单不可用: {manifest_path}；请先运行 prepare-environment。'
            ) from exc
        expected = {
            'gazebo_px4': 'udp',
            'port': self.transport['port'],
            'px4_source_dir': self.px4_source_dir,
        }
        actual = manifest.get('transport', {})
        if any(actual.get(key) != value for key, value in expected.items()):
            raise BatchConfigError(
                'UDP 环境清单与当前 YAML/PX4 不匹配；请重新运行 prepare-environment。'
            )
        expected_vehicle = {
            'world': self.world_path,
            'sensor': str(self.simulation.get('gazebo', {}).get('sensor', 'none')).lower(),
            'source_vehicle_sdf': self.sdf_path,
            'model_paths': self.gazebo_model_paths,
        }
        if any(manifest.get(key) != value for key, value in expected_vehicle.items()):
            raise BatchConfigError(
                'UDP 环境中的 world/传感器/整机 SDF 与当前 YAML 不匹配；'
                '请使用当前 YAML 重新运行 prepare-environment 并重启 Gazebo + MAVROS。'
            )

    def _prepare_px4_data_dir(self, working_dir: str) -> Tuple[str, str]:
        """Prepare the case-local PX4 data directory."""
        original_etc = os.path.join(self.px4_build_dir, 'etc')
        startup_script = self.transport['startup_script']
        if self.transport['gazebo_px4'] != 'udp' or startup_script:
            return original_etc, startup_script or 'etc/init.d-posix/rcS'

        data_dir = os.path.join(working_dir, 'px4_etc')
        shutil.copytree(original_etc, data_dir, symlinks=True)
        simulator_script = os.path.join(data_dir, 'init.d-posix', 'px4-rc.simulator')
        try:
            with open(simulator_script, 'r', encoding='utf-8') as stream:
                contents = stream.read()
        except OSError as exc:
            raise BatchConfigError(
                '该 PX4 build 不含 init.d-posix/px4-rc.simulator；请在 YAML 设置 '
                'px4.startup_script，显式启动 simulator_mavlink 的 UDP 模式。'
            ) from exc

        replacement = f'simulator_mavlink start -u {self.transport["port"]}'
        patched, count = re.subn(
            r'(?m)^\s*simulator_mavlink start -c [^\n]+$', replacement, contents,
        )
        if count != 1:
            raise BatchConfigError(
                '无法自动把该 PX4 的 simulator_mavlink 启动方式切换为 UDP；'
                '请在 YAML 设置 px4.startup_script。'
            )
        with open(simulator_script, 'w', encoding='utf-8') as stream:
            stream.write(patched)
        return data_dir, 'etc/init.d-posix/rcS'

    def _start_process(
        self, name: str, command: Sequence[str], log_name: str, cwd: Optional[str] = None
    ) -> _ManagedProcess:
        os.makedirs(self.log_dir, exist_ok=True)
        log_path = os.path.join(self.log_dir, log_name)
        with open(log_path, 'w', encoding='utf-8') as log_file:
            log_file.write('# command: ' + ' '.join(command) + '\n')
            log_file.flush()
            process = subprocess.Popen(
                list(command),
                cwd=cwd,
                env=self.environment,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        return _ManagedProcess(name=name, process=process, log_path=log_path)

    def _ros_service_available(self, service: str) -> bool:
        try:
            result = subprocess.run(
                ['rosservice', 'list'], env=self.environment,
                capture_output=True, text=True, timeout=3.0, check=False,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0 and service in result.stdout.splitlines()

    def _start_mavros_state_observer(self) -> None:
        if self._mavros_state is not None:
            return
        try:
            import rospy
            from mavros_msgs.msg import State
        except ImportError as exc:
            raise RuntimeError('缺少 rospy 或 mavros_msgs，无法观察 MAVROS 链路') from exc
        if not rospy.core.is_initialized():
            rospy.init_node('flight_eval_batch', anonymous=True, disable_signals=True)
        self._mavros_state = _MavrosStateObserver(rospy, State)

    def _wait_until(
        self, predicate: Callable[[], bool], timeout: float, label: str,
        processes: Iterable[Optional[_ManagedProcess]],
    ) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for process in processes:
                if process is not None:
                    process.ensure_running()
            if predicate():
                return
            time.sleep(0.25)
        raise RuntimeError(f'等待 {label} 超时 ({timeout:.1f}s)')

    def start(self) -> None:
        self._prepare()
        if self.transport['gazebo_px4'] == 'udp':
            self._verify_udp_environment()
        self._wait_until(
            lambda: self._ros_service_available('/gazebo/reset_world'),
            self.timeouts['gazebo'],
            '使用者启动的 Gazebo（缺少 /gazebo/reset_world）',
            [],
        )
        self._wait_until(
            lambda: self._ros_service_available('/mavros/set_mode'),
            self.timeouts['mavros'],
            '使用者启动的 MAVROS（缺少 /mavros/set_mode）',
            [],
        )
        self._start_mavros_state_observer()

    def _reset_world(self) -> None:
        try:
            result = subprocess.run(
                ['rosservice', 'call', '/gazebo/reset_world'], env=self.environment,
                capture_output=True, text=True, timeout=self.timeouts['reset'], check=False,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError('调用 /gazebo/reset_world 失败') from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f'/gazebo/reset_world 返回失败: {detail}')
        time.sleep(0.5)

    def _mavros_connected(self) -> bool:
        return bool(
            self._mavros_state
            and self._mavros_state.matches(
                True, self._mavros_transition_sequence, max_age=3.0,
            )
        )

    def _mavros_disconnected(self) -> bool:
        return bool(
            self._mavros_state
            and self._mavros_state.matches(
                False, self._mavros_transition_sequence, max_age=3.0,
            )
        )

    def _wait_for_px4_ready(self) -> None:
        """等待新的 MAVROS 链路和稳定心跳均就绪。"""
        processes = [self.px4_process]
        self._wait_until(
            self._mavros_connected, self.timeouts['px4'],
            'MAVROS 与新启动 PX4 的连接', processes,
        )
        stable_until = time.monotonic() + self.timeouts['px4_ready']
        while time.monotonic() < stable_until:
            for process in processes:
                if process is not None:
                    process.ensure_running()
            if not self._mavros_connected():
                raise RuntimeError('PX4 就绪稳定期内 MAVROS 连接中断')
            time.sleep(0.25)

    def start_px4(self, log_name: str, working_dir: str) -> None:
        if self.px4_process is not None:
            raise RuntimeError('PX4 已在运行')
        os.makedirs(working_dir, exist_ok=False)
        data_dir, startup_script = self._prepare_px4_data_dir(working_dir)
        if self._mavros_state is None:
            raise RuntimeError('MAVROS 状态观察器尚未启动')
        self._mavros_transition_sequence = self._mavros_state.sequence

        px4_command = [
            os.path.join(self.px4_build_dir, 'bin', 'px4'),
            '-d',
            data_dir,
            '-s', startup_script,
        ]
        self.px4_process = self._start_process(
            'PX4 SITL', px4_command, log_name, cwd=working_dir
        )
        self._wait_for_px4_ready()

    def stop_px4(self, wait_for_disconnect: bool = False) -> None:
        if self.px4_process is not None:
            if self._mavros_state is not None:
                self._mavros_transition_sequence = self._mavros_state.sequence
            managed_process = self.px4_process
            if not managed_process.stop():
                raise RuntimeError(
                    'PX4 进程组在 SIGKILL 后仍未退出；拒绝继续下一 case。'
                    f'日志: {managed_process.log_path}'
                )
            self.px4_process = None
            if wait_for_disconnect:
                self._wait_until(
                    self._mavros_disconnected, self.timeouts['mavros'],
                    'MAVROS 断开上一条 PX4', [],
                )

    def run_case(
        self,
        case: BatchCase,
        callback: Callable[[], Any],
        px4_working_dir: str,
    ) -> Any:
        if self._has_run_case:
            self._reset_world()
        self.start_px4(f'px4_{case.case_name}.log', px4_working_dir)
        try:
            return callback()
        finally:
            self.stop_px4(wait_for_disconnect=True)
            self._has_run_case = True

    def runtime_metadata(self) -> Dict[str, Any]:
        return {
            'px4': {
                'source_dir': self.px4_source_dir,
                'build_dir': self.px4_build_dir,
                'sim_model': self.px4_sim_model,
                'transport': copy.deepcopy(self.transport),
            },
            'gazebo': {
                **copy.deepcopy(self.gazebo_args),
                'world_resolved': self.world_path,
                'sdf_resolved': self.sdf_path,
            },
            'mavros': copy.deepcopy(self.mavros_args),
        }

    def close(self) -> None:
        try:
            self.stop_px4()
        finally:
            if self._mavros_state is not None:
                self._mavros_state.close()
                self._mavros_state = None


class FlightEvalBatchRunner:
    """执行一份已验证的 YAML 批次配置。"""

    def __init__(
        self,
        definition: BatchDefinition,
        continue_on_error: bool = False,
        result_handler: Optional[Callable[[Dict[str, Any], str], Any]] = None,
    ):
        self.definition = definition
        self.continue_on_error = continue_on_error
        self.result_handler = result_handler
        self._session: Optional[_SimulationSession] = None
        self._active_signature: Optional[str] = None
        self._summary_entries: List[Dict[str, Any]] = []

    @property
    def _summary_path(self) -> str:
        return os.path.join(self.definition.output_dir, 'batch_summary.json')

    def _write_json(self, path: str, data: Mapping[str, Any]) -> None:
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)

    def _write_summary(self, final: bool = False) -> None:
        counts: Dict[str, int] = {}
        for entry in self._summary_entries:
            status = entry['status']
            counts[status] = counts.get(status, 0) + 1
        payload = {
            'format_version': 1,
            'batch_name': self.definition.name,
            'config_file': self.definition.config_path,
            'output_dir': self.definition.output_dir,
            'analysis_enabled': self.definition.analysis_enabled,
            'final': final,
            'status_counts': counts,
            'cases': self._summary_entries,
        }
        self._write_json(self._summary_path, payload)

    def _prepare_output_dir(self) -> None:
        output_dir = self.definition.output_dir
        if os.path.exists(output_dir) and os.listdir(output_dir):
            raise BatchConfigError(
                f'批次输出目录已存在且非空，拒绝混合结果: {output_dir}'
            )
        os.makedirs(output_dir, exist_ok=True)
        shutil.copyfile(
            self.definition.config_path,
            os.path.join(output_dir, 'batch_config.yaml'),
        )
        archived_manifests = []
        if self.definition.manifest_paths:
            manifest_dir = os.path.join(output_dir, 'algorithm_manifests')
            os.makedirs(manifest_dir, exist_ok=True)
            for index, source in enumerate(self.definition.manifest_paths, start=1):
                filename = f'{index:02d}_{os.path.basename(source)}'
                destination = os.path.join(manifest_dir, filename)
                shutil.copyfile(source, destination)
                archived_manifests.append(os.path.relpath(destination, output_dir))
        self._write_json(
            os.path.join(output_dir, 'batch_plan.json'),
            {
                'format_version': 1,
                'batch_name': self.definition.name,
                'config_file': self.definition.config_path,
                'manifests': list(self.definition.manifest_paths),
                'archived_manifests': archived_manifests,
                'cases': [case.to_dict() for case in self.definition.cases],
            },
        )

    def _switch_session_if_needed(self, case: BatchCase) -> _SimulationSession:
        signature = case.simulation_signature
        if self._session is not None and signature == self._active_signature:
            return self._session
        if self._session is not None:
            self._session.close()
            self._session = None
        session_dir = os.path.join(self.definition.output_dir, 'logs', f'environment_{case.index:03d}')
        self._session = _SimulationSession(
            simulation=case.simulation,
            config_dir=os.path.dirname(self.definition.config_path),
            log_dir=session_dir,
        )
        self._session.start()
        self._active_signature = signature
        return self._session

    def _case_metadata(
        self,
        case: BatchCase,
        session: _SimulationSession,
        px4_working_dir: str,
    ) -> Dict[str, Any]:
        return {
            'batch': {
                'name': self.definition.name,
                'config_file': self.definition.config_path,
                'manifests': list(self.definition.manifest_paths),
                'case_index': case.index,
                'case_name': case.case_name,
                'experiment_name': case.experiment_name,
                'repetition': case.repetition,
                'px4_working_dir': px4_working_dir,
                'simulation': session.runtime_metadata(),
            },
        }

    def run(self) -> Dict[str, Any]:
        with _BatchTerminationGuard():
            return self._run()

    def _run(self) -> Dict[str, Any]:
        self._prepare_output_dir()
        print(format_batch_plan(self.definition))
        try:
            for case in self.definition.cases:
                record_dir = os.path.join(self.definition.output_dir, 'cases', case.case_name)
                px4_working_dir = os.path.join(record_dir, 'px4_rootfs')
                os.makedirs(record_dir, exist_ok=False)
                entry: Dict[str, Any] = case.to_dict()
                entry['record_dir'] = record_dir
                entry['started_at'] = datetime.now().isoformat(timespec='seconds')
                self._write_json(os.path.join(record_dir, 'resolved_case.json'), entry)

                try:
                    session = self._switch_session_if_needed(case)

                    def execute() -> Optional[Dict[str, Any]]:
                        run_options = case.run_options
                        duration = run_options.get('duration')
                        runner = FlightRunner(
                            controller_name=case.controller['name'],
                            task_name=case.task,
                            planner_name=case.planner['name'],
                            planner_args=case.planner['args'],
                            controller_args=case.controller['args'],
                            takeoff_height=float(run_options.get('takeoff_height', 2.0)),
                            duration=(float(duration) if duration is not None else None),
                            record_dir=record_dir,
                            auto_land=bool(run_options.get('auto_land', True)),
                            extra_topics=list(run_options.get('extra_topics', [])),
                            max_collision_episodes=int(
                                run_options.get('max_collision_episodes', 3)
                            ),
                            collision_episode_gap=float(
                                run_options.get('collision_episode_gap', 0.5)
                            ),
                            metadata_extra=self._case_metadata(
                                case, session, px4_working_dir,
                            ),
                        )
                        return runner.run()

                    result = session.run_case(case, execute, px4_working_dir)
                    if result is None:
                        entry['status'] = 'failed'
                        entry['error'] = 'FlightRunner 未产生结果；请查看 case 目录与环境日志。'
                    else:
                        entry['run_metadata_file'] = result.get('run_metadata_file', '')
                        entry['bag_file'] = result.get('bag_file', '')
                        entry['run_status'] = result.get('run_status', {})
                        entry['task_outcome'] = result.get('task_outcome', {})
                        entry['status'] = (
                            'completed'
                            if entry['run_status'].get('status') == 'completed'
                            else 'failed'
                        )
                        if entry['status'] == 'failed':
                            entry['error'] = (
                                'FlightRunner 未完整结束: '
                                f"{entry['run_status'].get('reason', 'unknown')}"
                            )
                        if self.definition.analysis_enabled and self.result_handler is not None:
                            try:
                                artifacts = self.result_handler(result, record_dir)
                                print(f'分析完成，生成的 artifacts: {artifacts}')
                                entry['analysis'] = artifacts or {}
                            except Exception as exc:  # keep a valid bag even when plotting fails
                                entry['analysis_error'] = str(exc)
                                entry['analysis_traceback'] = traceback.format_exc()
                    entry['finished_at'] = datetime.now().isoformat(timespec='seconds')
                except KeyboardInterrupt:
                    entry['status'] = 'interrupted'
                    entry['error'] = 'batch interrupted by SIGINT/SIGTERM'
                    entry['finished_at'] = datetime.now().isoformat(timespec='seconds')
                    self._summary_entries.append(entry)
                    self._write_summary(final=False)
                    raise
                except Exception as exc:
                    entry['status'] = 'failed'
                    entry['error'] = str(exc)
                    entry['traceback'] = traceback.format_exc()
                    entry['finished_at'] = datetime.now().isoformat(timespec='seconds')
                    if self._session is not None:
                        self._session.close()
                    self._session = None
                    self._active_signature = None

                self._summary_entries.append(entry)
                self._write_summary(final=False)
                if entry['status'] == 'failed' and not self.continue_on_error:
                    break
        finally:
            if self._session is not None:
                self._session.close()
                self._session = None
            self._write_summary(final=True)

        return {
            'summary_file': self._summary_path,
            'cases': copy.deepcopy(self._summary_entries),
        }
