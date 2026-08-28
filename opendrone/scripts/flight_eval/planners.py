#!/usr/bin/env python3
"""Built-in planner definitions."""

import copy
import math

_PLANNER_OUTPUT_TOPIC = '/planner/output'
_MISSION_STATE_TOPIC = '/planner/mission_state'
_AIRFAR_NATIVE_GOAL_TOLERANCE = 0.4
_DISCRETE_TASKS = ['discrete_circle', 'discrete_figure8']
_SEQUENTIAL_TASKS = _DISCRETE_TASKS + ['sequential_goal_mission']
_REFERENCE_PATH_TASKS = _DISCRETE_TASKS + ['reference_path_mission']


def _planner(
    description,
    launch_file,
    tasks,
    args=None,
    diagnostic_topics=None,
    runtime_parameter_expectations=None,
):
    evaluation_topics = {'planner_output': _PLANNER_OUTPUT_TOPIC}
    if any(task in tasks for task in {'sequential_goal_mission', 'reference_path_mission'}):
        evaluation_topics['mission_state'] = _MISSION_STATE_TOPIC
    return {
        'description': description,
        'launch_pkg': 'opendrone',
        'launch_file': launch_file,
        'args': dict(args or {}),
        'tasks': list(tasks),
        'evaluation_topics': evaluation_topics,
        'record_topics': list(dict.fromkeys(
            list(evaluation_topics.values()) + list(diagnostic_topics or [])
        )),
        'runtime_parameter_expectations': dict(
            runtime_parameter_expectations or {}
        ),
    }


PLANNER_REGISTRY = {
    'none': {
        'description': '不启动规划器',
        'launch_pkg': None,
        'launch_file': None,
        'args': {},
        'tasks': ['hover'],
        'evaluation_topics': {},
        'record_topics': [],
    },
    'analytic_reference': _planner(
        '解析参考发布器',
        'benchmarks/controller_benchmark/analytic_reference.launch',
        ['analytic_circle', 'analytic_figure8', 'analytic_spiral'],
        {'trajectory_type': 'circle'},
    ),
    'mav_trajectory': _planner(
        'mav_trajectory_generation',
        'benchmarks/trajectory_generation_benchmark/benchmark_mav_trajectory_planner.launch',
        _DISCRETE_TASKS,
        {'use_preset_waypoints': 'true', 'max_vel': '2.5', 'max_acc': '4'},
        ['/trajectory_generation/trajectory'],
    ),
    'rpg_trajectory': _planner(
        'RPG polynomial trajectory',
        'benchmarks/trajectory_generation_benchmark/benchmark_rpg_trajectory.launch',
        _DISCRETE_TASKS,
        {'use_preset_waypoints': 'true'},
        ['/rpg_traj/path'],
    ),
    'fast_planner_kino': _planner(
        'Fast-Planner kino',
        'benchmarks/online_planner_benchmark/benchmark_fast_planner.launch',
        _SEQUENTIAL_TASKS,
        {'use_preset_waypoints': 'true', 'use_kino_planner': 'true', 'use_rviz': 'false'},
        ['/planning/bspline', '/planning/pos_cmd'],
    ),
    'fast_planner_topo': _planner(
        'Fast-Planner topo',
        'benchmarks/online_planner_benchmark/benchmark_fast_planner.launch',
        _REFERENCE_PATH_TASKS,
        {'use_preset_waypoints': 'true', 'use_kino_planner': 'false', 'use_rviz': 'false'},
        ['/planning/bspline', '/planning/pos_cmd'],
    ),
    'ego_planner': _planner(
        'EGO-Planner depth camera',
        'benchmarks/online_planner_benchmark/benchmark_ego_planner.launch',
        _SEQUENTIAL_TASKS,
        {'use_preset_waypoints': 'true', 'use_rviz': 'false'},
        ['/drone_0_planning/bspline', '/drone_0_planning/pos_cmd'],
    ),
    'ego_planner_mid360': _planner(
        'EGO-Planner Mid360',
        'benchmarks/online_planner_benchmark/benchmark_ego_planner_mid360.launch',
        _SEQUENTIAL_TASKS,
        {'use_preset_waypoints': 'true', 'use_rviz': 'false'},
        ['/drone_0_planning/bspline', '/drone_0_planning/pos_cmd'],
    ),
    'ego_planner_v2': _planner(
        'EGO-Planner v2',
        'benchmarks/online_planner_benchmark/benchmark_ego_planner_v2.launch',
        _SEQUENTIAL_TASKS,
        {'use_preset_waypoints': 'true', 'use_rviz': 'false'},
        # EGO v2 publishes PolyTraj, not Bspline.  Recording the native
        # trajectory is required to verify adapter/native equivalence from a
        # failed run's rosbag.
        ['/drone_0_planning/trajectory', '/drone_0_planning/pos_cmd'],
    ),
    'super_planner_od': _planner(
        'SUPER planner OD',
        'benchmarks/online_planner_benchmark/benchmark_super_planner_od.launch',
        _SEQUENTIAL_TASKS,
        {
            'use_preset_waypoints': 'true',
            'use_rviz': 'false',
            'mission_waypoint_switch_distance': '1.0',
        },
        ['/planning_cmd/poly_traj', '/planning/pos_cmd'],
        runtime_parameter_expectations={
            '/super_planner_od_fsm/mission/waypoint_switch_distance': 1.0,
        },
    ),
    'airfar_planner': _planner(
        'Air-FAR depth camera',
        'benchmarks/online_planner_benchmark/benchmark_airfar_planner.launch',
        _SEQUENTIAL_TASKS,
        {
            'use_preset_waypoints': 'true',
            'use_rviz': 'false',
            'config_file': 'default',
            'cruise_speed': '0.5',
            'goal_tolerance': str(_AIRFAR_NATIVE_GOAL_TOLERANCE),
            'collision_check_padding': '1.0',
        },
        ['/way_point', '/path', '/track_path'],
        {
            '/terrainAnalysis/decayTime': 2.0,
            '/terrainAnalysisExt/decayTime': 10.0,
            '/airfar_local_planner/pathScale': 0.5,
            '/airfar_local_planner/minPathScale': 0.25,
            '/airfar_local_planner/collisionCheckPadding': 1.0,
            '/airfar_path_follower/cruiseSpeed': 0.5,
            '/airfar_path_follower/goalTolerance': 0.4,
            '/airfar_path_follower/maxSpeed': 1.5,
            '/airfar_planner/GPlanner/converge_distance': 0.4,
        },
    ),
    'airfar_planner_mid360': _planner(
        'Air-FAR Mid360',
        'benchmarks/online_planner_benchmark/benchmark_airfar_planner_mid360.launch',
        _SEQUENTIAL_TASKS,
        {
            'use_preset_waypoints': 'true',
            'use_rviz': 'false',
            'config_file': 'default',
            'cruise_speed': '1.0',
            'goal_tolerance': str(_AIRFAR_NATIVE_GOAL_TOLERANCE),
            'collision_check_padding': '1.0',
        },
        ['/way_point', '/path', '/track_path'],
        {
            '/terrainAnalysis/decayTime': 2.0,
            '/terrainAnalysisExt/decayTime': 10.0,
            '/airfar_local_planner/pathScale': 2.0,
            '/airfar_local_planner/minPathScale': 0.5,
            '/airfar_local_planner/collisionCheckPadding': 1.0,
            '/airfar_path_follower/cruiseSpeed': 1.0,
            '/airfar_path_follower/goalTolerance': 0.4,
            '/airfar_path_follower/maxSpeed': 1.5,
            '/airfar_planner/GPlanner/converge_distance': 0.4,
        },
    ),
}


def get_planner_launch(planner_name: str, extra_args=None) -> dict:
    if planner_name not in PLANNER_REGISTRY:
        raise ValueError(
            f"未知规划器: {planner_name}, 可用规划器: {list(PLANNER_REGISTRY)}"
        )
    info = copy.deepcopy(PLANNER_REGISTRY[planner_name])
    info['args'].update(extra_args or {})
    if planner_name in {'airfar_planner', 'airfar_planner_mid360'}:
        numeric_args = {
            'cruise_speed': [
                '/airfar_path_follower/cruiseSpeed',
            ],
            'goal_tolerance': [
                '/airfar_path_follower/goalTolerance',
                '/airfar_planner/GPlanner/converge_distance',
            ],
            'collision_check_padding': [
                '/airfar_local_planner/collisionCheckPadding',
            ],
        }
        for arg_name, parameter_names in numeric_args.items():
            if arg_name not in info['args']:
                continue
            try:
                value = float(info['args'][arg_name])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"{planner_name}.{arg_name} 必须是数值"
                ) from exc
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{planner_name}.{arg_name} 必须是有限正数"
                )
            for parameter_name in parameter_names:
                info['runtime_parameter_expectations'][parameter_name] = value
    if planner_name == 'super_planner_od':
        arg_name = 'mission_waypoint_switch_distance'
        try:
            value = float(info['args'][arg_name])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'{planner_name}.{arg_name} 必须是数值'
            ) from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(
                f'{planner_name}.{arg_name} 必须是有限正数'
            )
        info['runtime_parameter_expectations'][
            '/super_planner_od_fsm/mission/waypoint_switch_distance'
        ] = value
    return info
