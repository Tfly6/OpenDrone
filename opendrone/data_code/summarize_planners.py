"""Aggregate the frozen 300-flight benchmark by planner and sensor (stdlib only)."""
from pathlib import Path
import collections
import csv
import statistics
from revision_support.paths import data_root

HERE = Path(__file__).resolve().parent


def summarize():
    with (data_root() / 'revision_audit/trials.csv').open() as stream:
        trials = [r for r in csv.DictReader(stream)
                  if r['task'] in ('sequential_goal_mission', 'reference_path_mission')]
    groups = collections.defaultdict(list)
    for trial in trials:
        groups[(trial['sensor'], trial['planner'])].append(trial)
    rows = []
    for (sensor, planner), group in sorted(groups.items()):
        seq = [r for r in group if r['task'] == 'sequential_goal_mission']
        native = [r for r in group if r['native_status'] == 'succeeded']
        rows.append({
            'sensor': sensor, 'planner': planner, 'trials': len(group),
            'sequential_trials': len(seq),
            'reference_path_trials': len(group)-len(seq),
            'native_success': len(native),
            'native_success_no_recorded_contact': sum(float(r['runtime_contact_count']) == 0 for r in native),
            'native_success_with_recorded_contact': sum(float(r['runtime_contact_count']) > 0 for r in native),
            'sequential_geometric_completion_1m': sum(r['geometry_1.0'] == 'True' for r in seq),
            'sequential_native_success_geometric_incomplete_1m': sum(r['geometry_1.0'] == 'False' for r in native if r['task'] == 'sequential_goal_mission'),
            'median_reference_coverage': statistics.median(float(r['reference_coverage']) for r in group),
            'controllers': ';'.join(sorted({r['controller'] for r in group})),
        })
    assert sum(r['trials'] for r in rows) == 300
    assert sum(r['sequential_trials'] for r in rows) == 258
    assert sum(r['reference_path_trials'] for r in rows) == 42
    assert sum(r['native_success'] for r in rows) == 250
    assert sum(r['native_success_no_recorded_contact'] for r in rows) == 198
    assert sum(r['sequential_geometric_completion_1m'] for r in rows) == 219
    return rows


def main():
    rows = summarize()
    destination = data_root() / 'planner_summary.csv'
    with destination.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'Wrote {destination.name}: {len(rows)} configurations / 300 flights')


if __name__ == '__main__':
    main()
