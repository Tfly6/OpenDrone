#!/usr/bin/env python3
"""Load external algorithm manifests."""

from __future__ import annotations

import os
from typing import Any, Dict, Iterable, Mapping, MutableMapping

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


MANIFEST_VERSION = 1
MANIFEST_SCHEMA_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'schemas',
    'algorithm_manifest.schema.yaml',
)

_ALGORITHM_KEYS = {'kind', 'launch', 'args', 'tasks', 'record_topics'}
_LAUNCH_KEYS = {'package', 'file'}
_STANDARD_INTERFACES = {
    'controller': {'flight_state': '/flight_state'},
    'planner': {'planner_output': '/planner/output'},
}


class ManifestError(ValueError):
    pass


def _mapping(value: Any, label: str) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ManifestError(f'{label} 必须是 mapping')
    return dict(value)


def _string_list(value: Any, label: str) -> list:
    if value is None:
        return []
    if not isinstance(value, list) or not all(
        isinstance(item, str) and item for item in value
    ):
        raise ManifestError(f'{label} 必须是非空字符串 list')
    return list(dict.fromkeys(value))


def _ros_arg(value: Any) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if value is None:
        return ''
    return str(value)


def _normalise_algorithm(name: str, value: Any, source: str) -> tuple:
    if not isinstance(name, str) or not name:
        raise ManifestError(f'{source} 的算法名称必须是非空字符串')
    raw = _mapping(value, f'{source}.{name}')
    unknown = set(raw) - _ALGORITHM_KEYS
    if unknown:
        raise ManifestError(f'{source}.{name} 包含未知字段: {sorted(unknown)}')

    kind = raw.get('kind')
    if kind not in _STANDARD_INTERFACES:
        raise ManifestError(f'{source}.{name}.kind 必须是 controller 或 planner')

    launch = _mapping(raw.get('launch'), f'{source}.{name}.launch')
    unknown_launch = set(launch) - _LAUNCH_KEYS
    if unknown_launch:
        raise ManifestError(
            f'{source}.{name}.launch 包含未知字段: {sorted(unknown_launch)}'
        )
    package = launch.get('package')
    launch_file = launch.get('file')
    if not isinstance(package, str) or not package:
        raise ManifestError(f'{source}.{name}.launch.package 必须是非空字符串')
    if not isinstance(launch_file, str) or not launch_file:
        raise ManifestError(f'{source}.{name}.launch.file 必须是非空字符串')

    args = _mapping(raw.get('args'), f'{source}.{name}.args')
    if not all(isinstance(key, str) and key for key in args):
        raise ManifestError(f'{source}.{name}.args 的 key 必须是非空字符串')
    invalid_args = [
        key for key, value in args.items()
        if isinstance(value, (Mapping, list))
    ]
    if invalid_args:
        raise ManifestError(
            f'{source}.{name}.args 只接受字符串、数值、布尔值或 null: {invalid_args}'
        )
    args = {key: _ros_arg(value) for key, value in args.items()}
    tasks = _string_list(raw.get('tasks'), f'{source}.{name}.tasks')
    record_topics = _string_list(
        raw.get('record_topics'), f'{source}.{name}.record_topics'
    )
    invalid_topics = [topic for topic in record_topics if not topic.startswith('/')]
    if invalid_topics:
        raise ManifestError(
            f'{source}.{name}.record_topics 必须使用绝对 ROS topic: {invalid_topics}'
        )

    interfaces = dict(_STANDARD_INTERFACES[kind])
    entry = {
        'description': name,
        'launch_pkg': package,
        'launch_file': launch_file,
        'args' if kind == 'planner' else 'launch_args': args,
        'tasks': tasks,
        'evaluation_topics': interfaces,
        'record_topics': list(dict.fromkeys(
            list(interfaces.values()) + record_topics
        )),
        'manifest_path': source,
        'manifest_source': f'{source}:{name}',
    }
    return kind, entry


def load_manifest(
    path: str,
    controller_registry: MutableMapping[str, Dict],
    planner_registry: MutableMapping[str, Dict],
) -> Dict[str, list]:
    if yaml is None:
        raise RuntimeError('算法 manifest 需要 PyYAML，请安装: pip install PyYAML')
    resolved = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(resolved):
        raise ManifestError(f'找不到算法 manifest: {resolved}')
    try:
        with open(resolved, 'r', encoding='utf-8') as stream:
            root = yaml.safe_load(stream) or {}
    except yaml.YAMLError as exc:
        raise ManifestError(f'manifest YAML 解析失败: {resolved}: {exc}') from exc
    root = _mapping(root, resolved)
    unknown = set(root) - {'version', 'algorithms'}
    if unknown:
        raise ManifestError(f'{resolved} 根对象包含未知字段: {sorted(unknown)}')
    if root.get('version') != MANIFEST_VERSION:
        raise ManifestError(
            f'{resolved} 只支持 version: {MANIFEST_VERSION}，'
            f'得到: {root.get("version")!r}'
        )
    algorithms = _mapping(root.get('algorithms'), f'{resolved}.algorithms')
    if not algorithms:
        raise ManifestError(f'{resolved}.algorithms 必须是非空 mapping')

    staged = []
    loaded = {'controller': [], 'planner': []}
    from .tasks import TASK_REGISTRY

    for name, raw in algorithms.items():
        kind, entry = _normalise_algorithm(name, raw, resolved)
        unknown_tasks = set(entry['tasks']) - set(TASK_REGISTRY)
        if unknown_tasks:
            raise ManifestError(
                f'{resolved}.{name}.tasks 包含未知任务: {sorted(unknown_tasks)}'
            )
        registry = controller_registry if kind == 'controller' else planner_registry
        if name in registry:
            if registry[name].get('manifest_source') == entry['manifest_source']:
                continue
            raise ManifestError(f'{resolved} 重复定义 {kind} {name}')
        staged.append((name, kind, entry))

    for name, kind, entry in staged:
        registry = controller_registry if kind == 'controller' else planner_registry
        registry[name] = entry
        loaded[kind].append(name)
    return loaded


def load_manifest_files(
    paths: Iterable[str],
    controller_registry: MutableMapping[str, Dict],
    planner_registry: MutableMapping[str, Dict],
) -> Dict[str, list]:
    loaded = {'controller': [], 'planner': []}
    for path in paths:
        result = load_manifest(path, controller_registry, planner_registry)
        loaded['controller'].extend(result['controller'])
        loaded['planner'].extend(result['planner'])
    return loaded
