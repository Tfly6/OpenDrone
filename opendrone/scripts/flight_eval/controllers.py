#!/usr/bin/env python3
"""Built-in controller definitions."""

import copy


def _controller(description, launch_file, record_topics):
    return {
        'description': description,
        'launch_pkg': 'opendrone',
        'launch_file': launch_file,
        'launch_args': {},
        'tasks': [],
        'evaluation_topics': {'flight_state': '/flight_state'},
        'record_topics': list(dict.fromkeys(['/flight_state'] + record_topics)),
    }


CONTROLLER_REGISTRY = {
    'px4_position_baseline': _controller(
        'PX4 内置位置控制 baseline',
        'sitl_base_controller.launch',
        ['/mavros/setpoint_position/local'],
    ),
    'se3_hopf': _controller(
        'SE(3) 几何控制器 (Hopf)',
        'benchmarks/controller_benchmark/benchmark_se3_hopf.launch',
        ['/mavros/setpoint_raw/attitude', '/mavros/setpoint_position/local'],
    ),
    'se3_lee': _controller(
        'SE(3) 几何控制器 (Lee)',
        'benchmarks/controller_benchmark/benchmark_se3_lee.launch',
        ['/mavros/setpoint_raw/attitude', '/mavros/setpoint_position/local'],
    ),
    'lqr_controller': _controller(
        'LQR 控制器',
        'benchmarks/controller_benchmark/benchmark_lqr_controller.launch',
        ['/mavros/setpoint_raw/attitude', '/mavros/setpoint_position/local'],
    ),
    'pid_controller': _controller(
        '级联 PID 控制器',
        'benchmarks/controller_benchmark/benchmark_pid_controller.launch',
        [
            '/mavros/setpoint_position/local',
            '/mavros/setpoint_raw/local',
            '/mavros/setpoint_raw/attitude',
        ],
    ),
    'mav_linear_mpc': _controller(
        '线性 MPC 控制器',
        'benchmarks/controller_benchmark/benchmark_mpc_controller.launch',
        ['/command/roll_pitch_yawrate_thrust', '/mavros/setpoint_raw/attitude'],
    ),
    'mav_nonlinear_mpc': _controller(
        '非线性 MPC 控制器',
        'benchmarks/controller_benchmark/benchmark_nmpc_controller.launch',
        ['/command/roll_pitch_yawrate_thrust', '/mavros/setpoint_raw/attitude'],
    ),
}


def get_controller_launch(controller_name: str, extra_args=None) -> dict:
    if controller_name not in CONTROLLER_REGISTRY:
        raise ValueError(
            f"未知控制器: {controller_name}, 可用控制器: {list(CONTROLLER_REGISTRY)}"
        )
    info = copy.deepcopy(CONTROLLER_REGISTRY[controller_name])
    info['launch_args'].update(extra_args or {})
    return info
