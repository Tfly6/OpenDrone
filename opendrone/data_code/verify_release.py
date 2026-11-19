"""Verify frozen evidence totals and recompute the worked example from CSV.

Only Python standard-library modules are required.
"""
import collections
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
from revision_support.paths import data_root

HERE = Path(__file__).resolve().parent


def read_csv(relative):
    return list(csv.DictReader((data_root() / Path(relative).relative_to('data')).open()))


def percentile(values, fraction):
    values = sorted(values)
    q = (len(values)-1)*fraction
    i = int(q)
    return values[i] + (values[min(i+1,len(values)-1)]-values[i])*(q-i)


# Source files included in the reproducibility checksum manifest.
ROOT_FILES = {
    'README.md', 'DATA_README.md', 'requirements.txt', '.gitignore',
    'verify_release.py', 'summarize_planners.py', 'build_evidence.py', 'export_worked_example.py',
    'draw_framework.py', 'evidence_manifest.json', 'run_revision.py', 'run_revision.sh',
    'record_ground_truth.py', 'revision_manifest.json', 'run_suite.py', 'run_suite.sh', 'verify_example.py',
}


def release_files():
    result = []
    for path in HERE.rglob('*'):
        if not path.is_file() or '__pycache__' in path.parts or 'build' in path.parts:
            continue
        rel = path.relative_to(HERE)
        if (len(rel.parts) == 1 and path.name in ROOT_FILES
                or rel.parts[0] in {'samples', 'experiments', 'revision_support', 'checks'}):
            result.append(path)
    return sorted(result)


def main():
    trials = read_csv('data/revision_audit/trials.csv')
    online = [r for r in trials if r['task'] in ('sequential_goal_mission', 'reference_path_mission')]
    assert len(trials) == 566
    assert sum(r['candidate_included'] == 'True' for r in trials) == 561
    assert len(online) == 300
    assert sum(r['native_status'] == 'succeeded' for r in online) == 250
    assert sum(r['native_status'] == 'succeeded' and float(r['runtime_contact_count']) == 0 for r in online) == 198
    seq = [r for r in online if r['task'] == 'sequential_goal_mission']
    bad = [r for r in seq if r['native_status'] == 'succeeded' and r['geometry_1.0'] == 'False']
    assert len(bad) == 5 and sum(float(r['runtime_contact_count']) == 0 for r in bad) == 4
    assert collections.Counter(r['waypoint_count'] for r in seq) == {'1':172,'2':86}
    from summarize_planners import summarize
    expected_summary = [{key: str(value) for key, value in row.items()} for row in summarize()]
    assert read_csv('data/planner_summary.csv') == expected_summary
    quality = read_csv('data/revision_review_20261116/trial_quality.csv')
    assert len(quality) == 80 and all(r['odom_geometry_1.0'] == r['world_truth_geometry_1.0'] == 'True' for r in quality)
    assert sum(float(r['runtime_contacts']) == 0 for r in quality) == 79
    assert all(r['runtime_contacts'] == r['replay_contacts'] for r in quality)
    combos = read_csv('data/online_combinations.csv')
    assert len(combos) == 300 and len({r['combination_id'] for r in combos}) == 50
    fixture = read_csv('data/revision_validation/validation.csv')
    assert len(fixture) == 18 and all(r['passed'] == 'True' for r in fixture)
    labels = collections.Counter(label for r in fixture for label in r['expected_labels'].split(';') if label)
    assert labels == {
        'native_success_without_geometric_fulfillment':4,
        'geometric_fulfillment_without_native_success':1,
        'geometric_evidence_unknown':3,
        'recorded_contact':4,
        'reference_coverage_below_0.9':1,
        'contact_evidence_unknown':1,
    }
    assert sum(not r['expected_labels'] for r in fixture) == 4
    reversals = collections.Counter((r['odom_geometry_0.5'],r['world_truth_geometry_0.5'])
                                    for r in quality if r['odom_geometry_0.5'] != r['world_truth_geometry_0.5'])
    assert reversals == {('False','True'):4,('True','False'):1}
    signal = read_csv('data/worked_example/aligned_task.csv')
    # Independently reconstruct position support from exported raw points.
    expected = json.loads((data_root()/'worked_example/readout.json').read_text())
    raw_points = read_csv('data/worked_example/reference_points.csv')
    windows = []
    for point in raw_points:
        if point['index'] == '0':
            windows.append({'received':float(point['bag_time']), 'points':[]})
        if int(point['valid_mask']) & 1:
            windows[-1]['points'].append((float(point['trajectory_start'])+float(point['time_from_start']), [float(point[k]) for k in ('x','y','z')]))
    windows = [w for w in windows if w['points']]
    for w in windows:
        w['points'].sort()
    receipt = [w['received'] for w in windows]
    reconstructed = []
    for row in read_csv('data/worked_example/state.csv'):
        t = float(row['bag_time'])
        if not expected['start'] <= t <= expected['end']:
            continue
        j = bisect.bisect_right(receipt, t)-1
        ref = None
        if j >= 0:
            points = windows[j]['points']
            if len(points) == 1:
                ref = points[0][1]
            elif points[0][0] <= t <= points[-1][0]:
                k = min(max(bisect.bisect_right([p[0] for p in points],t)-1,0),len(points)-2)
                left,right = points[k],points[k+1]
                ratio = (t-left[0])/(right[0]-left[0]) if right[0] != left[0] else 0
                ref = [a+ratio*(b-a) for a,b in zip(left[1],right[1])]
        if ref is not None:
            reconstructed.append(math.sqrt(sum((float(row[key])-ref[i])**2 for i,key in enumerate(('x','y','z')))))
    assert len(reconstructed) == 860
    assert abs(math.sqrt(sum(x*x for x in reconstructed)/len(reconstructed))-expected['rmse_m']) < 1e-10
    errors = []
    for row in signal:
        values = [float(row[k]) for k in ('x','y','z','ref_x','ref_y','ref_z')]
        if all(math.isfinite(v) for v in values):
            errors.append(math.sqrt(sum((values[i]-values[i+3])**2 for i in range(3))))
    assert len(signal) == 930 and len(errors) == 860
    assert abs(math.sqrt(sum(v*v for v in errors)/len(errors))-expected['rmse_m']) < 1e-10
    assert abs(percentile(errors,.95)-expected['p95_m']) < 1e-10
    assert min(float(r['goal_distance_m']) for r in signal) > 2
    assert len(read_csv('data/worked_example/contacts.csv')) == 2
    files = release_files()
    (HERE/'release_checksums.json').write_text(json.dumps({str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)},indent=2,ensure_ascii=False))
    print('Verified: 561 benchmark + 80 controlled trials; 50 combinations; 8 planner configurations; 18 rule records; independent raw-reference reconstruction.')


if __name__ == '__main__':
    main()
