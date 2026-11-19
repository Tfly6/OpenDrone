"""Review completed revision batches and check raw truth/odometry support."""
import argparse
import collections
from pathlib import Path

import numpy as np

from .audit import read_json, write_json, write_csv, ordered_visitation
from .paths import workspace_root, artifacts_root


def review(workspace, output):
    import rosbag
    workspace, output = Path(workspace), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows, summaries = [], []
    for directory in sorted((artifacts_root(workspace) / 'revision_runs').iterdir()):
        completed = directory / 'completed_batch.json'
        if not completed.exists():
            continue
        batch = workspace / 'eval_runs/batches' / Path(read_json(completed)['output_dir']).name
        summary = read_json(batch / 'batch_summary.json')
        design = read_json(directory / 'design.json')
        batch_rows = []
        for case in summary['cases']:
            case_dir = batch / 'cases' / case['case_name']
            cache = directory / 'audit/trials' / batch.name / (case_dir.name + '.json')
            if not cache.exists():
                rows.append({'batch': batch.name, 'case': case_dir.name, 'error': 'audit cache missing'})
                continue
            replay = read_json(cache)['replay']
            report = read_json(case_dir / 'report.json')
            metadata = read_json(case_dir / 'run_metadata.json')
            metrics = {m['name']: m for m in report['metrics']}
            start, end = replay['start'], replay['end']
            odom, truth, durations = [], [], []
            with rosbag.Bag(str(case_dir / 'flight_test.bag')) as bag:
                for topic, msg, stamp in bag.read_messages(topics=[
                    '/mavros/local_position/odom', '/flight_eval/ground_truth/odom',
                    '/flight_eval/ground_truth/query_duration']):
                    if not start <= stamp.to_sec() <= end:
                        continue
                    if topic.endswith('query_duration'):
                        durations.append(msg.data)
                        continue
                    p = msg.pose.pose.position
                    value = [msg.header.stamp.to_sec(), p.x, p.y, p.z]
                    (truth if 'ground_truth' in topic else odom).append(value)
            row = {'batch': batch.name, 'case': case_dir.name, 'controller': case['controller']['name'],
                   'planner': case['planner']['name'], 'repetition': case['repetition'],
                   'status': case['status'], 'native': report['task_outcome']['status'],
                   'task_instance_hash': replay['task_instance_hash'],
                   'waypoint_count': len(replay['goals']), 'mission_start': start, 'mission_end': end,
                   'archived_execution_duration_s': report['execution_duration'],
                   'archived_coverage': metrics['reference_coverage_ratio']['value'],
                   'mission_coverage': replay['tracking']['reference_coverage_ratio']['value'],
                   'odometry_samples': len(odom), 'truth_samples': len(truth),
                   'runtime_contacts': replay['runtime_contact_count'],
                   'replay_contacts': len(replay['contact_replay']['mission']),
                   'post_mission_contacts': len(replay['contact_replay']['post_mission']),
                   'analysis_error': case.get('analysis_error', ''),
                   'cleanup_completed': metadata['process_cleanup']['completed']}
            for tolerance, result in replay['geometric_results'].items():
                row['odom_geometry_' + tolerance] = result['complete']
            if odom and truth:
                state, true = np.asarray(odom), np.asarray(truth)
                row['state_clock_backwards'] = int(np.count_nonzero(np.diff(state[:, 0]) < 0))
                row['truth_clock_backwards'] = int(np.count_nonzero(np.diff(true[:, 0]) < 0))
                row['truth_max_gap_s'] = float(np.max(np.diff(true[:, 0]))) if len(true) > 1 else None
                order = np.argsort(true[:, 0], kind='stable')
                true = true[order]
                idx = np.searchsorted(true[:, 0], state[:, 0])
                left, right = np.clip(idx-1, 0, len(true)-1), np.clip(idx, 0, len(true)-1)
                nearest = np.where(abs(true[left, 0]-state[:, 0]) <= abs(true[right, 0]-state[:, 0]), left, right)
                dt = abs(true[nearest, 0]-state[:, 0])
                keep = dt <= 0.05
                row['truth_match_fraction_50ms'] = float(np.mean(keep))
                if np.any(keep):
                    errors = state[keep, 1:] - true[nearest[keep], 1:]
                    bias = np.mean(errors, axis=0)
                    row['raw_odom_truth_rmse_m'] = float(np.sqrt(np.mean(np.sum(errors**2, axis=1))))
                    row['raw_odom_truth_p95_m'] = float(np.percentile(np.linalg.norm(errors, axis=1), 95))
                    row['raw_bias_x_m'], row['raw_bias_y_m'], row['raw_bias_z_m'] = map(float, bias)
                    row['timestamp_nearest_p95_s'] = float(np.percentile(dt[keep], 95))
                for tolerance in (0.5, 1.0, 2.0):
                    row['world_truth_geometry_' + str(tolerance)] = ordered_visitation(
                        true[:, 1:], true[:, 0], replay['goals'], tolerance)['complete']
            row['query_duration_p95_s'] = float(np.percentile(durations, 95)) if durations else None
            row['query_duration_max_s'] = float(max(durations)) if durations else None
            rows.append(row)
            batch_rows.append(row)
        log = (directory / 'environment.log').read_text(errors='replace')
        summaries.append({'batch': batch.name, 'plan_count': len(design['ordered_cases']),
                          'cases': len(summary['cases']), 'reports': len(batch_rows),
                          'statuses': dict(collections.Counter(c['status'] for c in summary['cases'])),
                          'native_success': sum(r['native'] == 'succeeded' for r in batch_rows),
                          'odom_geometric_completion_1m': sum(r['odom_geometry_1.0'] is True for r in batch_rows),
                          'truth_geometric_completion_1m': sum(r.get('world_truth_geometry_1.0') is True for r in batch_rows),
                          'no_recorded_contact': sum(r['runtime_contacts'] == 0 for r in batch_rows),
                          'contact_count_disagreements': sum(r['runtime_contacts'] != r['replay_contacts'] for r in batch_rows),
                          'truth_samples_present': sum(r['truth_samples'] > 0 for r in batch_rows),
                          'task_file_sha256': design['task_sha256'],
                          'thread_deadlock_in_environment_log': 'Resource deadlock avoided' in log,
                          'timing_restart_log_count': log.count('Time jump detected')})
        print(f'Reviewed {batch.name}: {len(batch_rows)} raw bags', flush=True)
    write_csv(output / 'trial_quality.csv', rows)
    write_json(output / 'summary.json', {
        'batches': summaries,
        'matching_policy': 'nearest ground-truth header stamp within 50ms; no spatial alignment fitted',
        'geometry_policy': 'raw world-frame truth versus task coordinates; declared identity world/map transform',
        'scope': 'direct raw-coordinate discrepancy includes frame-origin offset and time-sampling error; does not establish estimator accuracy independently'})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default=str(workspace_root()))
    parser.add_argument('--output', default=str(artifacts_root() / 'revision_review_20261116'))
    args = parser.parse_args()
    review(args.workspace, args.output)
