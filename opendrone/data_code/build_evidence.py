"""Build manuscript tables, plots and a hashed evidence index from frozen audits.

This command reads existing audits; it never starts SITL or edits experiment logs.
"""
import collections
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import statistics
import zipfile
import argparse
from revision_support.paths import data_root

os.environ.setdefault('MPLCONFIGDIR', '/tmp/opendrone_manuscript_mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
INPUT_ROOT = HERE
LABELS = {'px4_position_baseline': 'PX4', 'se3_hopf': 'Hopf', 'se3_lee': 'Lee',
          'lqr_controller': 'LQR', 'pid_controller': 'PID', 'mav_linear_mpc': 'MPC',
          'mav_nonlinear_mpc': 'NMPC', 'ego_planner': 'EGO', 'ego_planner_v2': 'EGO-v2',
          'fast_planner_kino': 'Fast-kino', 'fast_planner_topo': 'Fast-topo',
          'airfar_planner': 'Air-FAR', 'ego_planner_mid360': 'EGO-L',
          'airfar_planner_mid360': 'Air-FAR-L', 'super_planner_od': 'SUPER',
          'mav_trajectory': 'MAV', 'rpg_trajectory': 'RPG', 'none': 'Analytic'}


def read_csv(path):
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))


def num(value):
    return float(value) if value not in ('', None) else None


def esc(value):
    return str(value).replace('\\', r'\textbackslash{}').replace('_', r'\_').replace('%', r'\%').replace('&', r'\&').replace('#', r'\#')


def table(name, headers, rows, widths=None):
    spec = widths or ('l' * len(headers))
    text = r'\begin{longtable}{' + spec + '}\n\\toprule\n'
    text += ' & '.join(headers) + r' \\ \midrule' + '\n\\endhead\n'
    text += '\n'.join(' & '.join(str(x) for x in row) + r' \\' for row in rows)
    text += '\n\\bottomrule\n\\end{longtable}\n'
    (HERE / 'generated' / (name + '.tex')).write_text(text)


def main():
    global HERE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=INPUT_ROOT / 'outputs/evidence')
    args = parser.parse_args()
    HERE = args.output.expanduser().resolve()
    input_data = data_root()
    if HERE == INPUT_ROOT or INPUT_ROOT in HERE.parents and HERE.parts[len(INPUT_ROOT.parts)] == 'data':
        parser.error('Output must not overwrite the frozen package/data directory')
    for folder in ('assets', 'data', 'generated'):
        (HERE / folder).mkdir(parents=True, exist_ok=True)
    sources = []
    for folder, names in {
        'revision_audit': ['trials.csv', 'joint_outcomes.csv', 'coverage_sensitivity.csv',
                           'plan_to_result.csv', 'summary.json', 'provenance.json'],
        'revision_review_20261116': ['trial_quality.csv', 'summary.json'],
        'revision_validation': ['fixtures.json', 'validation.csv', 'protocol_ablation.csv', 'summary.json'],
    }.items():
        for name in names:
            src = input_data / folder / name
            dest = HERE / 'data' / folder / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
            sources.append(src)
    rows = read_csv(HERE / 'data/revision_audit/trials.csv')
    quality = read_csv(HERE / 'data/revision_review_20261116/trial_quality.csv')
    joint = read_csv(HERE / 'data/revision_audit/joint_outcomes.csv')
    online = [r for r in rows if r['task'] in ('sequential_goal_mission', 'reference_path_mission')]
    assert len(rows) == 566 and sum(r['candidate_included'] == 'True' for r in rows) == 561
    assert len(online) == 300 and len(quality) == 80
    assert sum(r['native_status'] == 'succeeded' for r in online) == 250
    assert sum(r['native_status'] == 'succeeded' and num(r['runtime_contact_count']) == 0 for r in online) == 198

    # Aggregate eight-cell joint counts; path fulfillment retains unknown.
    counter = collections.Counter()
    for r in joint:
        if r['tolerance_m'] == '1.0':
            counter[(r['task_group'], r['native_status'], r['geometric_completion'], r['runtime_recorded_contact'])] += int(r['count'])
    jrows = []
    for group, label in [('single_goal', r'\lang{单点}{Single goal}'), ('multi_goal', r'\lang{双点}{Two goals}'), ('reference_path', r'\lang{路径}{Path}')]:
        for native in ('succeeded', 'not_succeeded'):
            vals = []
            for geom in ('True', 'False', 'unknown'):
                for contact in ('none_recorded', 'recorded'):
                    vals.append(counter[(group, native, geom, contact)])
            jrows.append([label, '+' if native == 'succeeded' else '-', *vals])
    table('joint', [r'\lang{任务}{Task}', 'N', r'G+ C0', 'G+ C1', 'G- C0', 'G- C1', 'G? C0', 'G? C1'], jrows, 'llrrrrrr')

    # All 50 algorithm combinations, their task-instance hashes and repeats.
    combinations = collections.defaultdict(list)
    for r in online:
        combinations[(r['sensor'], r['controller'], r['planner'], r['task'])].append(r)
    assert len(combinations) == 50
    combo_rows = []
    with (HERE / 'data/online_combinations.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['combination_id', 'sensor', 'controller', 'planner', 'task', 'trial_id', 'world', 'repetition', 'task_instance_hash'])
        for i, (key, records) in enumerate(sorted(combinations.items()), 1):
            for r in records:
                writer.writerow([f'C{i:02d}', *key, r['trial_id'], r['world'], r['repetition'], r['task_instance_hash']])
            combo_rows.append([f'C{i:02d}', 'Depth' if key[0] == 'depth_camera' else 'Mid360', LABELS[key[1]], LABELS[key[2]], 'path' if key[3] == 'reference_path_mission' else 'goals', len(records)])
    table('combinations', ['ID', r'\lang{传感器}{Sensor}', r'\lang{控制器}{Controller}', r'\lang{规划器}{Planner}', r'\lang{任务类型}{Task type}', 'n'], combo_rows)
    planner_joint = []
    for r in joint:
        if r['tolerance_m'] == '1.0':
            planner_joint.append([r['task_group'].replace('single_goal', 'single').replace('multi_goal', 'two').replace('reference_path', 'path'), LABELS[r['planner']], '+' if r['native_status'] == 'succeeded' else '-', {'True': '+', 'False': '-', 'unknown': '?'}.get(r['geometric_completion'], '?'), '0' if r['runtime_recorded_contact'] == 'none_recorded' else '1', r['count']])
    table('joint_by_planner', [r'\lang{任务}{Task}', r'\lang{规划器}{Planner}', 'N', 'G', 'C', 'n'], planner_joint, 'lllllr')

    # Sample retention and coverage-conditioned summaries, always using archived windows.
    tracking = [r for r in rows if r['candidate_included'] == 'True' and r['task'].startswith(('analytic_', 'discrete_'))]
    groups = collections.defaultdict(list)
    for r in tracking:
        groups[(r['controller'], r['task'], r['planner'])].append(r)
    retention, sensitivity = [], []
    for (controller, task, planner), records in sorted(groups.items()):
        ns = sum(int(num(r['state_samples']) or 0) for r in records)
        nr = sum(int(num(r['rmse_samples']) or 0) for r in records)
        np95 = sum(int(num(r['p95_samples']) or 0) for r in records)
        assert nr == np95
        retention.append([LABELS[controller], task.replace('analytic_', 'A-').replace('discrete_', 'D-'), LABELS[planner], len(records), ns, nr, f'{nr/ns:.3f}' if ns else '--', f'{min(num(r["retained_fraction"]) for r in records):.3f}'])
        for threshold in (0, .8, .9, .95):
            kept = [r for r in records if num(r['reference_coverage']) >= threshold]
            sensitivity.append([controller, task, planner, threshold, len(records), len(kept), statistics.median(num(r['rmse_m']) for r in kept) if kept else '', statistics.median(num(r['p95_m']) for r in kept) if kept else ''])
    table('retention', [r'\lang{控制器}{Ctrl}', r'\lang{任务}{Task}', r'\lang{生成器}{Gen}', 'n', r'$N_s$', r'$N_e$', r'$N_e/N_s$', r'\lang{最小比例}{Min fraction}'], retention, 'lllrrrrr')
    with (HERE / 'data/tracking_sensitivity.csv').open('w', newline='') as stream:
        w = csv.writer(stream); w.writerow(['controller', 'task', 'planner', 'coverage_threshold', 'original_n', 'retained_n', 'median_rmse_m', 'median_p95_m']); w.writerows(sensitivity)
    sensitivity_table = []
    for key, records in sorted(groups.items()):
        variants = [[r for r in records if num(r['reference_coverage']) >= threshold] for threshold in (0, .9, .95)]
        row = [LABELS[key[0]], key[1].replace('analytic_', 'A-').replace('discrete_', 'D-'), LABELS[key[2]]]
        for kept in variants:
            row.extend([len(kept), f'{statistics.median(num(r["rmse_m"]) for r in kept):.3f}' if kept else '--'])
        sensitivity_table.append(row)
    table('tracking_sensitivity', [r'\lang{控制器}{Ctrl}', r'\lang{任务}{Task}', r'\lang{生成器}{Gen}', r'$n_0$', r'$E_0$', r'$n_{.9}$', r'$E_{.9}$', r'$n_{.95}$', r'$E_{.95}$'], sensitivity_table, 'lllrrrrrr')
    fixtures = read_csv(HERE / 'data/revision_validation/validation.csv')
    fixture_rows = [[r'\path{' + r['id'] + '}', str(r['geometry']) or '?', r['contact_count'], r'\path{' + (r['labels'] or 'none') + '}'] for r in fixtures]
    table('fixtures', [r'\lang{案例}{Case}', 'G', r'$n_C$', r'\lang{诊断}{Diagnosis}'], fixture_rows, 'p{.30\\linewidth}llp{.49\\linewidth}')
    coverage_rows = []
    for task in sorted({r['task'] for r in tracking}):
        rr = [r for r in tracking if r['task'] == task]
        coverage_rows.append([task.replace('analytic_', 'A-').replace('discrete_', 'D-'), len(rr), f'{min(num(r["reference_coverage"]) for r in rr):.3f}', *[sum(num(r['reference_coverage']) >= t for r in rr) for t in (.8, .9, .95)]])
    table('coverage', [r'\lang{任务}{Task}', 'n', r'\lang{最小覆盖}{Min cov.}', r'$\geq0.80$', r'$\geq0.90$', r'$\geq0.95$'], coverage_rows, 'lrrrrr')

    # Derived statistics and full per-trial audit data remain separate from old reports.
    disagreement = [r for r in online if num(r['runtime_contact_count']) != num(r['bag_mission_contact_count'])]
    alignment_changed = []
    # Frozen derived statistics carry the historical full-cache check. Do not
    # claim to have repeated it when only the distributable CSV set is present.
    historical_stats = json.loads((input_data / 'derived_statistics.json').read_text())
    alignment_changed = historical_stats['alignment_factor_sensitivity_changed']
    raw = [num(r['raw_odom_truth_rmse_m']) for r in quality]
    stats = {'archived_reports': len(rows), 'candidate_included': 561, 'new_trials': len(quality),
             'runtime_replay_contact_count_disagreements_online': len(disagreement),
             'alignment_factor_sensitivity_changed': alignment_changed,
             'alignment_sensitivity_source': 'frozen derived_statistics.json; full bag/cache check not rerun',
             'raw_odom_truth_rmse_min_m': min(raw), 'raw_odom_truth_rmse_median_m': statistics.median(raw), 'raw_odom_truth_rmse_max_m': max(raw),
             'truth_odom_label_disagreement': {t: sum(r['odom_geometry_'+t] != r['world_truth_geometry_'+t] for r in quality) for t in ('0.5', '1.0', '2.0')},
             'tracking_coverage_counts': coverage_rows}
    for batch in sorted({r['batch'] for r in quality}):
        rr = [r for r in quality if r['batch'] == batch]
        stats[batch] = {key: {'min': min(num(r[key]) for r in rr), 'median': statistics.median(num(r[key]) for r in rr)} for key in ('archived_coverage', 'mission_coverage')}
    (HERE / 'data/derived_statistics.json').write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'pdf.fonttype': 42, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7), constrained_layout=True)
    for ax, batch, label in zip(axes, sorted({r['batch'] for r in quality}), ['Cylinders', 'Forest']):
        rr = [r for r in quality if r['batch'] == batch]
        x = np.array([num(r['archived_coverage']) for r in rr]); y = np.array([num(r['mission_coverage']) for r in rr])
        ax.scatter(x, y, s=20, color='#216b78', alpha=.75); ax.plot([.7, 1], [.7, 1], ':', color='gray')
        ax.set(xlim=(.7, 1.02), ylim=(.7, 1.02), xlabel='Controller-active coverage', ylabel='Task-window coverage', title=label)
    fig.savefig(HERE / 'assets/window_coverage.pdf'); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(7, 2.7), constrained_layout=True)
    for group, color in [('single_goal', '#216b78'), ('multi_goal', '#ad6539')]:
        vals = []
        for tolerance in (.5, 1, 2):
            vals.append(sum(int(r['count']) for r in joint if r['task_group'] == group and float(r['tolerance_m']) == tolerance and r['native_status'] == 'succeeded' and r['geometric_completion'] == 'False'))
        axes[0].plot([.5, 1, 2], vals, 'o-', label=group.replace('_', ' '), color=color)
    axes[0].set(xlabel='Visitation tolerance (m)', ylabel='Native success, geometry incomplete'); axes[0].legend(frameon=False)
    axes[1].hist(raw, bins=12, color='#216b78', edgecolor='white'); axes[1].set(xlabel='Raw odometry / truth RMSE (m)', ylabel='Trials')
    fig.savefig(HERE / 'assets/geometry_truth.pdf'); plt.close(fig)
    import draw_framework
    draw_framework.HERE = HERE
    draw_framework.draw('en'); draw_framework.draw('zh')

    # Hash the inputs used in this evidence build.
    manifest = {'edition': 'OpenDrone portable data-code evidence rebuild',
                'historical_corpus': 'reconstructed from locally archived reports and audited bags',
                'raw_bags': 'external workspace archives; not copied into this source package',
                'sources': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources]}
    (HERE / 'evidence_manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False))

    # Small configuration/evaluator archives accompany the trial index; bags stay external.
    for name in ('configuration_records.zip', 'replay_source_snapshot.zip'):
        shutil.copy2(input_data / name, HERE / 'data' / name)


if __name__ == '__main__':
    main()
