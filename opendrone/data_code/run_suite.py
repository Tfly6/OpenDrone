"""List, plan and run the full paper experiment matrix (646 attempts).

`list` requires only Python's standard library. `plan` resolves ROS launch
files without starting a simulator. `run` starts the selected flight batches.
"""
import argparse
import collections
import copy
from datetime import datetime
import json
from pathlib import Path
import signal

from revision_support.paths import PACKAGE_ROOT, artifacts_root, px4_source, workspace_root

GROUPS = ('hover', 'tracking', 'generation', 'online', 'controlled', 'all')


def load_suite():
    return json.loads((PACKAGE_ROOT / 'experiments/full_suite.json').read_text())


def select_batches(suite, group='all', sensor=None, world=None):
    selected = []
    for batch in suite['batches']:
        cases = []
        for case in batch['cases']:
            profile = suite['profiles'][case['profile']]
            sim = profile['simulation']['gazebo']
            if group != 'all' and case['group'] != group:
                continue
            if sensor and sim.get('sensor') != sensor:
                continue
            if world and sim.get('world') != world:
                continue
            cases.append(case)
        if cases:
            selected.append(dict(batch, cases=cases))
    return selected


def make_definition(suite, batch, directory, px4=None, limit=None):
    from flight_eval.batch import BatchCase, BatchDefinition
    from revision_support.audit import write_json
    import yaml
    directory.mkdir(parents=True, exist_ok=True)
    cases = []
    for record in batch['cases'][:limit]:
        profile = copy.deepcopy(suite['profiles'][record['profile']])
        simulation = profile['simulation']
        simulation['px4']['source_dir'] = str(Path(px4).expanduser().resolve() if px4 else px4_source())
        simulation['transport']['environment_dir'] = str(directory / 'environment')
        planner = profile['planner']
        if profile['waypoint_parameters']:
            task_file = directory / 'tasks' / (record['profile'] + '.yaml')
            task_file.parent.mkdir(parents=True, exist_ok=True)
            task_file.write_text(yaml.safe_dump(profile['waypoint_parameters'], sort_keys=True))
            planner['args']['_paper_task_config'] = str(task_file)
        cases.append(BatchCase(index=record['index'], experiment_name=record['name'],
                               repetition=record['repetition'], task=profile['task'],
                               controller=profile['controller'], planner=planner,
                               run_options=profile['run_options'], simulation=simulation))
    config = directory / 'suite_plan.json'
    write_json(config, {'format': 'explicit_paper_trial_plan_v1', 'source_batch': batch['id'],
                        'runner': 'run_suite.py', 'cases': [c.to_dict() for c in cases]})
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return BatchDefinition(name='paper_' + batch['id'], config_path=str(config),
                           output_dir=str(workspace_root() / 'eval_runs/batches' / ('paper_' + batch['id'] + '_' + stamp)),
                           analysis_enabled=True, cases=cases, manifest_paths=[])


def validate_definition(definition):
    from revision_support.suite_runtime import SuiteFlightRunner
    import roslaunch.config
    seen = set()
    for case in definition.cases:
        key = json.dumps((case.task, case.controller, case.planner), sort_keys=True)
        if key in seen:
            continue
        seen.add(key)
        runner = SuiteFlightRunner(controller_name=case.controller['name'], task_name=case.task,
                                   planner_name=case.planner['name'], planner_args=case.planner['args'],
                                   controller_args=case.controller['args'],
                                   takeoff_height=case.run_options.get('takeoff_height', 2),
                                   duration=case.run_options.get('duration'),
                                   record_dir=str(Path(definition.output_dir) / 'cases' / case.case_name))
        for info in (runner.ctrl_info, runner.planner_info):
            if not info.get('launch_file'):
                continue
            path = runner._resolve_launch_path(info['launch_pkg'], info['launch_file'])
            arguments = [f'{k}:={v}' for k, v in info.get('launch_args', info.get('args', {})).items()]
            config = roslaunch.config.load_config_default([(path, arguments)], port=None,
                                                           verbose=False, assign_machines=False)
            if info is runner.planner_info and runner.paper_waypoints:
                for name, expected in runner.paper_waypoints.items():
                    parameter = config.params.get('/waypoint_generator/' + name)
                    if parameter is None or parameter.value != expected:
                        raise ValueError(f'{case.case_name}: waypoint parameter mismatch: {name}')
    print(f'Validated {len(seen)} task/controller/planner launch configurations', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('list', 'plan', 'run'))
    parser.add_argument('group', nargs='?', choices=GROUPS, default='all',
                        help='Experiment design: controller regulation/tracking, generation, online scenario packages, or controlled fixed-task comparisons')
    parser.add_argument('--sensor', choices=('none', 'depth_camera', 'Mid360'))
    parser.add_argument('--world', help='e.g. empty, cylinders_85_45, forest, cylinders_dynamic_40_30')
    parser.add_argument('--limit', type=int, help='Take at most N trials per selected batch (smoke run)')
    parser.add_argument('--px4-source')
    args = parser.parse_args()
    if args.limit is not None and args.limit < 1:
        parser.error('--limit must be positive')
    suite = load_suite()
    batches = select_batches(suite, args.group, args.sensor, args.world)
    if not batches:
        parser.error('No experiment matches the selection')
    counts = collections.Counter(c['group'] for b in batches for c in b['cases'])
    print(json.dumps({'attempts': sum(counts.values()), 'groups': {g: counts[g] for g in GROUPS if g in counts},
                      'batches': {b['id']:len(b['cases']) for b in batches}}, indent=2), flush=True)
    if args.action == 'list':
        return
    # Imports initialize the current checkout's optional ROS experiment overlay.
    from run_revision import execute_definition
    from revision_support.suite_runtime import SuiteFlightRunner
    import flight_eval.batch
    for batch in batches:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        category = 'suite_plans' if args.action == 'plan' else 'suite_runs'
        directory = artifacts_root() / category / (batch['id'] + '_' + stamp)
        definition = make_definition(suite, batch, directory, args.px4_source, args.limit)
        validate_definition(definition)
        print(f'Prepared {len(definition.cases)} trials: {directory}', flush=True)
        if args.action == 'run':
            previous = flight_eval.batch.FlightRunner
            flight_eval.batch.FlightRunner = SuiteFlightRunner
            try:
                execute_definition(definition, directory)
            finally:
                flight_eval.batch.FlightRunner = previous


if __name__ == '__main__':
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
