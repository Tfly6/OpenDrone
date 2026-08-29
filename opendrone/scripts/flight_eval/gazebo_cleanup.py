#!/usr/bin/env python3
"""Best-effort shutdown helpers for Gazebo Classic environments."""

from __future__ import annotations

import os
import signal
import subprocess
import time
import xmlrpc.client
from typing import Dict, Iterable, List, Set


_GAZEBO_NODE_NAMES = ('/gazebo', '/gazebo_gui')
_ENVIRONMENT_LAUNCH_HINTS = (
    '/launch/flight_eval/environment.launch',
    'flight_eval/environment.launch',
)


def _lookup_ros_node_pids(node_names: Iterable[str]) -> Dict[str, int]:
    try:
        import rosgraph
    except ImportError:
        return {}

    master = rosgraph.Master('/flight_eval_gazebo_cleanup')
    pids: Dict[str, int] = {}
    for node_name in node_names:
        try:
            uri = master.lookupNode(node_name)
            code, _message, pid = xmlrpc.client.ServerProxy(uri).getPid(
                '/flight_eval_gazebo_cleanup'
            )
        except Exception:
            continue
        if code == 1 and isinstance(pid, int) and pid > 0:
            pids[node_name] = pid
    return pids


def _find_environment_roslaunch_pids() -> Set[int]:
    try:
        result = subprocess.run(
            ['ps', '-eo', 'pid=,args='],
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return set()
    if result.returncode != 0:
        return set()

    pids: Set[int] = set()
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, args = stripped.partition(' ')
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        if 'roslaunch' not in args:
            continue
        if any(hint in args for hint in _ENVIRONMENT_LAUNCH_HINTS):
            pids.add(pid)
    return pids


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _wait_gone(pids: Iterable[int], timeout: float) -> Set[int]:
    deadline = time.monotonic() + timeout
    remaining = {pid for pid in pids if pid > 0}
    while remaining and time.monotonic() < deadline:
        remaining = {pid for pid in remaining if _pid_exists(pid)}
        if remaining:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    return {pid for pid in remaining if _pid_exists(pid)}


def _signal_pids(pids: Iterable[int], sig: signal.Signals) -> Set[int]:
    failed: Set[int] = set()
    for pid in sorted(set(pids)):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            continue
        except PermissionError:
            failed.add(pid)
    return failed


def shutdown_gazebo_simulation() -> Dict[str, object]:
    """Stop the Gazebo Classic environment attached to the current ROS master.

    The cleanup is intentionally scoped:
    - use ROS XML-RPC to find `/gazebo` and `/gazebo_gui` pids;
    - stop roslaunch processes that launched OpenDrone's flight_eval environment;
    - avoid global `pkill gzserver/gzclient`, which can kill unrelated simulators.
    """
    node_pids = _lookup_ros_node_pids(_GAZEBO_NODE_NAMES)
    launch_pids = _find_environment_roslaunch_pids()
    pids = set(node_pids.values()) | launch_pids
    report: Dict[str, object] = {
        'requested': True,
        'node_pids': node_pids,
        'environment_roslaunch_pids': sorted(launch_pids),
        'remaining_pids': [],
        'completed': True,
        'errors': [],
    }
    if not pids:
        return report

    denied: Set[int] = set()
    for sig, timeout in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 3.0),
        (signal.SIGKILL, 2.0),
    ):
        denied |= _signal_pids(pids, sig)
        remaining = _wait_gone(pids, timeout)
        if not remaining:
            residual_node_pids = _lookup_ros_node_pids(_GAZEBO_NODE_NAMES)
            report['remaining_pids'] = sorted(residual_node_pids.values())
            report['completed'] = not denied and not residual_node_pids
            if denied:
                report['errors'] = [
                    f'permission denied while signaling pids: {sorted(denied)}'
                ]
            if residual_node_pids:
                report['errors'] = list(report.get('errors', [])) + [
                    f'gazebo nodes still registered after cleanup: {residual_node_pids}'
                ]
            return report
        pids = remaining

    report['remaining_pids'] = sorted(pids)
    report['completed'] = False
    errors: List[str] = []
    if denied:
        errors.append(f'permission denied while signaling pids: {sorted(denied)}')
    if pids:
        errors.append(f'pids still alive after SIGKILL: {sorted(pids)}')
    report['errors'] = errors
    return report
