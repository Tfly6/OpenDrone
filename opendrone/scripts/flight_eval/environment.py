#!/usr/bin/env python3
"""为外部 Gazebo/MAVROS 环境生成与指定 PX4 版本匹配的 UDP 模型覆盖。"""

from __future__ import annotations

import json
import os
import shlex
import shutil
import xml.etree.ElementTree as ET
from typing import Any, Dict, Tuple

from .batch import BatchConfigError, BatchDefinition, _SimulationSession

_PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
)
_OPENDRONE_ROOT = os.path.dirname(_PACKAGE_ROOT)
_WORKSPACE_ROOT = os.path.dirname(os.path.dirname(_OPENDRONE_ROOT))


def _write_text(path: str, contents: str) -> None:
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write(contents)


def _ensure_contact_plugin(model: ET.Element) -> None:
    """向整机 model 注入一次轻量 contact 插件配置。"""
    contact_plugin = next(
        (
            plugin for plugin in model.findall('plugin')
            if plugin.get('name') == 'opendrone_contact'
        ),
        None,
    )
    if contact_plugin is None:
        contact_plugin = ET.SubElement(
            model,
            'plugin',
            {
                'name': 'opendrone_contact',
                'filename': 'libopendrone_contact_plugin.so',
            },
        )
    contact_settings = {
        'robotNamespace': '/flight_eval',
        'topicName': 'contact_pulse',
        'recontactGap': '0.1',
        'ignoredCollisionSubstring': 'ground_plane',
    }
    for key, value in contact_settings.items():
        element = contact_plugin.find(key)
        if element is None:
            element = ET.SubElement(contact_plugin, key)
        element.text = value


def _patch_iris_udp(source_model_dir: str, output_model_dir: str, port: int) -> str:
    """复制当前 PX4 的完整 Iris 模型，切为 UDP 并注入轻量碰撞插件。"""
    source_sdf = os.path.join(source_model_dir, 'iris.sdf')
    source_config = os.path.join(source_model_dir, 'model.config')
    if not os.path.isfile(source_sdf) or not os.path.isfile(source_config):
        raise BatchConfigError(
            f'PX4 Iris 模型不完整: {source_model_dir}（需要 iris.sdf 和 model.config）'
        )

    shutil.copytree(source_model_dir, output_model_dir, dirs_exist_ok=True, symlinks=True)
    output_sdf = os.path.join(output_model_dir, 'iris.sdf')
    try:
        tree = ET.parse(output_sdf)
    except ET.ParseError as exc:
        raise BatchConfigError(f'无法解析 PX4 Iris SDF: {output_sdf}') from exc

    target_plugin = None
    for plugin in tree.getroot().iter('plugin'):
        if plugin.get('name') == 'mavlink_interface':
            target_plugin = plugin
            break
    if target_plugin is None:
        raise BatchConfigError('PX4 Iris SDF 不含 mavlink_interface，无法生成 UDP 覆盖模型')

    use_tcp = target_plugin.find('use_tcp')
    if use_tcp is None:
        use_tcp = ET.SubElement(target_plugin, 'use_tcp')
    use_tcp.text = '0'
    udp_port = target_plugin.find('mavlink_udp_port')
    if udp_port is None:
        udp_port = ET.SubElement(target_plugin, 'mavlink_udp_port')
    udp_port.text = str(port)

    model = tree.getroot().find('model')
    if model is None:
        raise BatchConfigError(f'PX4 Iris SDF 不含 model: {output_sdf}')
    _ensure_contact_plugin(model)

    tree.write(output_sdf, encoding='utf-8', xml_declaration=True)
    return output_sdf


def _patch_vehicle_sdf_udp(source_sdf: str, output_sdf: str, port: int) -> Tuple[str, bool]:
    """复制自定义整机 SDF 并改为 UDP，同时返回是否使用 Iris 覆盖。"""
    try:
        tree = ET.parse(source_sdf)
    except ET.ParseError as exc:
        raise BatchConfigError(f'无法解析 Gazebo SDF: {source_sdf}') from exc

    iris_include = any(
        (include.findtext('uri') or '').strip() == 'model://iris'
        for include in tree.getroot().iter('include')
    )
    plugins = [
        plugin for plugin in tree.getroot().iter('plugin')
        if plugin.get('name') == 'mavlink_interface'
    ]
    if iris_include:
        return source_sdf, True
    if not plugins:
        raise BatchConfigError(
            f'自定义 gazebo.sdf 不含 model://iris 或 mavlink_interface: {source_sdf}；'
            '整机模型需采用其中一种方式才能接入 PX4 UDP。'
        )

    os.makedirs(os.path.dirname(output_sdf), exist_ok=True)
    for plugin in plugins:
        use_tcp = plugin.find('use_tcp')
        if use_tcp is None:
            use_tcp = ET.SubElement(plugin, 'use_tcp')
        use_tcp.text = '0'
        udp_port = plugin.find('mavlink_udp_port')
        if udp_port is None:
            udp_port = ET.SubElement(plugin, 'mavlink_udp_port')
        udp_port.text = str(port)
    model = tree.getroot().find('model')
    if model is None:
        raise BatchConfigError(f'自定义 Gazebo SDF 不含 model: {source_sdf}')
    _ensure_contact_plugin(model)
    tree.write(output_sdf, encoding='utf-8', xml_declaration=True)
    return output_sdf, False


def prepare_environment(definition: BatchDefinition) -> Dict[str, Any]:
    """生成用户 source 后用于手动启动 Gazebo/MAVROS 的环境文件。"""
    if not definition.cases:
        raise BatchConfigError('batch 没有可准备的样本')
    case = definition.cases[0]
    session = _SimulationSession(
        simulation=case.simulation,
        config_dir=os.path.dirname(definition.config_path),
        log_dir=os.path.join(definition.output_dir, 'prepare_logs'),
    )
    session._prepare()
    if session.transport['gazebo_px4'] != 'udp':
        raise BatchConfigError('prepare-environment 目前只用于 transport.gazebo_px4: udp')
    environment_dir = session.transport['environment_dir']
    if not environment_dir:
        raise BatchConfigError('UDP 模式需要 transport.environment_dir')

    overlay_root = os.path.join(environment_dir, 'model_overrides')
    iris_output_dir = os.path.join(overlay_root, 'iris')
    iris_source_dir = os.path.join(
        session.px4_source_dir, 'Tools', 'simulation', 'gazebo-classic',
        'sitl_gazebo-classic', 'models', 'iris',
    )
    os.makedirs(environment_dir, exist_ok=True)
    output_sdf = _patch_iris_udp(
        iris_source_dir, iris_output_dir, session.transport['port'],
    )
    gazebo_config = session.simulation.get('gazebo', {})
    sensor = str(gazebo_config.get('sensor', 'none')).lower()
    if sensor in {'none', 'iris', 'default'} and not gazebo_config.get('sdf'):
        launch_sdf = output_sdf
        uses_iris_overlay = True
    else:
        vehicle_output = os.path.join(environment_dir, 'vehicle_overrides', 'vehicle.sdf')
        launch_sdf, uses_iris_overlay = _patch_vehicle_sdf_udp(
            session.sdf_path, vehicle_output, session.transport['port'],
        )

    px4_models = os.path.join(
        session.px4_source_dir, 'Tools', 'simulation', 'gazebo-classic',
        'sitl_gazebo-classic', 'models',
    )
    opendrone_models = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', 'sitl_config', 'models'
    ))
    plugin_dir = os.path.join(session.px4_build_dir, 'build_gazebo-classic')
    opendrone_plugin_dir = os.path.join(_WORKSPACE_ROOT, 'devel', 'lib')
    shell_path = os.path.join(environment_dir, 'environment.env')
    model_paths = [overlay_root, *session.gazebo_model_paths, opendrone_models, px4_models]
    model_path_text = ':'.join(shlex.quote(path) for path in model_paths)
    plugin_path_text = ':'.join(
        shlex.quote(path) for path in (opendrone_plugin_dir, plugin_dir)
    )
    _write_text(shell_path, '\n'.join((
        '# Generated by flight_eval prepare-environment. Source this before roslaunch.',
        f'export GAZEBO_MODEL_PATH={model_path_text}${{GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}}',
        f'export GAZEBO_PLUGIN_PATH={plugin_path_text}${{GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}}',
        f'export LD_LIBRARY_PATH={plugin_path_text}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}',
        '',
    )))

    launch_file = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', 'launch', 'flight_eval',
        'environment.launch',
    ))
    launch_args = {
        'world': session.world_path,
        'sdf': launch_sdf,
        **{key: value for key, value in session.gazebo_args.items()
           if key not in {'world', 'sdf', 'respawn_gazebo'}},
        **{key: value for key, value in session.mavros_args.items()
           if key != 'respawn_mavros'},
    }
    launch_command = 'roslaunch ' + ' '.join(
        [shlex.quote(launch_file)]
        + [f'{key}:={shlex.quote(str(value).lower() if isinstance(value, bool) else str(value))}'
           for key, value in launch_args.items()]
    )
    manifest = {
        'format_version': 1,
        'transport': {
            'gazebo_px4': 'udp',
            'port': session.transport['port'],
            'px4_source_dir': session.px4_source_dir,
        },
        'overlay_model_path': overlay_root,
        'patched_iris_sdf': output_sdf,
        'world': session.world_path,
        'sensor': sensor,
        'source_vehicle_sdf': session.sdf_path,
        'vehicle_sdf': launch_sdf,
        'uses_iris_overlay': uses_iris_overlay,
        'model_paths': session.gazebo_model_paths,
        'contact_plugin_path': opendrone_plugin_dir,
        'launch_command': launch_command,
    }
    manifest_path = os.path.join(environment_dir, 'environment.json')
    with open(manifest_path, 'w', encoding='utf-8') as stream:
        json.dump(manifest, stream, indent=2, ensure_ascii=False)
    _write_text(os.path.join(environment_dir, 'launch_command.sh'), launch_command + '\n')
    return {
        'environment_dir': environment_dir,
        'shell_file': shell_path,
        'manifest_file': manifest_path,
        'launch_command_file': os.path.join(environment_dir, 'launch_command.sh'),
        'launch_command': launch_command,
    }
