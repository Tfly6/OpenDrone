#!/usr/bin/env python3
"""Resolve the parameter targets created by a ROS1 launch invocation."""

from __future__ import annotations

import os
from typing import Dict, Mapping, Optional


def _canonical_name(name: str) -> str:
    parts = [part for part in str(name).split('/') if part]
    return '/' + '/'.join(parts) if parts else '/'


def inspect_launch(
    package: Optional[str],
    launch_file: str,
    args: Mapping[str, str],
    resolved_path: Optional[str] = None,
) -> Dict[str, object]:
    import rosgraph.names
    import roslaunch.config
    import roslaunch.rlutil

    arg_tokens = [f'{key}:={value}' for key, value in args.items()]
    if resolved_path and os.path.isfile(resolved_path):
        launch_path = os.path.abspath(resolved_path)
    else:
        if not package:
            raise ValueError(f'无法解析 launch 且没有包名: {launch_file}')
        resolved = roslaunch.rlutil.resolve_launch_arguments(
            [package, launch_file] + arg_tokens
        )
        launch_path = resolved[0]
        arg_tokens = resolved[1:]

    config = roslaunch.config.load_config_default(
        [(launch_path, arg_tokens)],
        port=None,
        verbose=False,
        assign_machines=False,
    )
    node_namespaces = sorted({
        _canonical_name(rosgraph.names.ns_join(node.namespace, node.name))
        for node in config.nodes
    })
    declared_parameters = sorted({
        _canonical_name(name) for name in config.params
        if _canonical_name(name) not in {'/rosdistro', '/rosversion'}
    })
    launch_files = sorted({
        os.path.abspath(path) for path in config.roslaunch_files
        if os.path.abspath(path) != os.path.abspath(
            roslaunch.config.get_roscore_filename()
        )
    })
    return {
        'package': package or '',
        'file': launch_file,
        'resolved_path': launch_path,
        'args': dict(args),
        'node_namespaces': node_namespaces,
        'declared_parameters': declared_parameters,
        'launch_files': launch_files,
    }


def is_within_namespace(parameter: str, namespace: str) -> bool:
    parameter = _canonical_name(parameter)
    namespace = _canonical_name(namespace)
    return parameter == namespace or parameter.startswith(namespace + '/')
