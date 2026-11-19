"""Export sufficient time-series evidence for the worked Fast-kino example.

Run after sourcing ROS Noetic and the catkin workspace. Output is read-only
with respect to the experiment archive.
"""
import csv
import hashlib
import json
from pathlib import Path
import sys
import argparse
from revision_support.paths import workspace_root, SCRIPTS_ROOT, artifacts_root

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = workspace_root()
sys.path.insert(0, str(SCRIPTS_ROOT))
from flight_eval.analyzer import BagAnalyzer
from flight_eval.metrics import MetricsCalculator
import rosbag


def write_csv(path, headers, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream); writer.writerow(headers); writer.writerows(rows)


def main():
    global ROOT, HERE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=ROOT)
    parser.add_argument('--output', type=Path, default=HERE / 'outputs/worked_example_export')
    args = parser.parse_args()
    ROOT = args.workspace.expanduser().resolve()
    HERE = args.output.expanduser().resolve()
    (HERE / 'assets').mkdir(parents=True, exist_ok=True)
    relative = Path('eval_runs/batches/dynamic_depth_20260828_222921/cases/048_sequential_goal_mission1_pid_controller_fast_planner_kino_r02')
    case = ROOT / relative
    cached = artifacts_root(ROOT) / 'revision_audit/trials' / relative.parts[2] / (case.name + '.json')
    audit = json.loads(cached.read_text())['replay']
    output = HERE / 'data/worked_example'
    output.mkdir(parents=True, exist_ok=True)
    for name in ('report.json', 'resolved_case.json', 'run_metadata.json', 'parameter_snapshot.json', 'rosparams.json'):
        (output / name).write_bytes((case / name).read_bytes())
    (output / 'audit.json').write_text(json.dumps(audit, indent=2))
    start, end = audit['start'], audit['end']
    analyzer = BagAnalyzer(str(case / 'flight_test.bag'), 'pid_controller', task_name='sequential_goal_mission', planner_name='fast_planner_kino', hover_height=2, duration=120)
    data = analyzer._extract_bag_data()
    states, points, missions, contacts = [], [], [], []
    with rosbag.Bag(str(case / 'flight_test.bag')) as bag:
        for topic, msg, stamp in bag.read_messages(topics=['/mavros/local_position/odom', '/planner/output', '/planner/mission_state', '/flight_eval/contact_pulse']):
            t = stamp.to_sec()
            if topic == '/mavros/local_position/odom':
                p = msg.pose.pose.position
                states.append([t, msg.header.stamp.to_sec(), msg.header.frame_id, p.x, p.y, p.z])
            elif topic == '/planner/output':
                for k, point in enumerate(msg.points):
                    p = point.position
                    points.append([t, msg.trajectory_start_time.to_sec(), msg.trajectory_id, k, point.time_from_start.to_sec(), point.valid_mask, p.x, p.y, p.z])
            elif topic == '/planner/mission_state':
                missions.append([t, msg.mission_type, msg.status, msg.completed_items, msg.total_items])
            else:
                contacts.append([t, msg.other_collision, msg.self_collision])
    write_csv(output / 'state.csv', ['bag_time', 'header_time', 'frame', 'x', 'y', 'z'], states)
    write_csv(output / 'reference_points.csv', ['bag_time', 'trajectory_start', 'trajectory_id', 'index', 'time_from_start', 'valid_mask', 'x', 'y', 'z'], points)
    write_csv(output / 'native_states.csv', ['bag_time', 'mission_type', 'status', 'completed_items', 'total_items'], missions)
    write_csv(output / 'contacts.csv', ['bag_time', 'other_collision', 'self_collision'], contacts)
    keep = (data['position_times'] >= start) & (data['position_times'] <= end)
    times, positions = data['position_times'][keep], data['positions'][keep]
    refs = {float(t): p for t, p in zip(data['ref_pos_times'], data['reference_positions'])}
    target = np.asarray(audit['goals'][-1])
    trajectory = []
    for t, p in zip(times, positions):
        ref = refs.get(float(t), np.full(3, np.nan))
        trajectory.append([t, t-start, *p, *ref, np.linalg.norm(p-target)])
    write_csv(output / 'aligned_task.csv', ['bag_time', 'elapsed', 'x', 'y', 'z', 'ref_x', 'ref_y', 'ref_z', 'goal_distance_m'], trajectory)
    valid_ref = (data['ref_pos_times'] >= start) & (data['ref_pos_times'] <= end)
    metrics = MetricsCalculator().compute_position_tracking_metrics(positions, times, data['reference_positions'][valid_ref], data['ref_pos_times'][valid_ref])
    assert abs(metrics[0].value-audit['tracking']['position_tracking_rmse_3d']['value']) < 1e-10
    result = {'trial_id': str(relative), 'start': start, 'end': end, 'state_samples': len(times),
              'valid_tracking_samples': metrics[0].detail['sample_count'], 'rmse_m': metrics[0].value,
              'p95_m': metrics[1].value, 'goal': target.tolist(),
              'minimum_goal_distance_m': float(np.min(np.linalg.norm(positions-target, axis=1))),
              'final_goal_distance_m': float(np.linalg.norm(positions[-1]-target)),
              'native_terminal_states': missions, 'recorded_contacts': contacts,
              'bag_sha256': hashlib.sha256((case/'flight_test.bag').read_bytes()).hexdigest()}
    (output/'readout.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a = np.array(trajectory)
    fig, axes = plt.subplots(2, 1, figsize=(7, 3.8), sharex=True, constrained_layout=True)
    axes[0].plot(a[:, 1], a[:, -1], color='#216b78', label='Distance to goal')
    axes[0].axhline(1, color='gray', linestyle=':', label='1 m reporting threshold')
    axes[0].set(ylabel='Goal distance (m)'); axes[0].legend(frameon=False, fontsize=8)
    errors = np.linalg.norm(a[:, 2:5]-a[:, 5:8], axis=1)
    axes[1].plot(a[:, 1], errors, color='#216b78', label='Tracking error; gaps retained')
    axes[1].set(xlabel='Time since task start (s)', ylabel='Tracking error (m)')
    for ax in axes:
        for i, contact in enumerate(contacts):
            ax.axvline(contact[0]-start, color='#ad6539', linestyle='--', label='Recorded contact' if i == 0 else None)
        ax.axvline(end-start, color='#8f3b4b', linestyle='-', label='Native terminal / window end')
        ax.spines[['top','right']].set_visible(False)
    axes[1].legend(frameon=False, fontsize=7, loc='upper left')
    fig.savefig(HERE/'assets/worked_timeline.pdf')


if __name__ == '__main__':
    main()
