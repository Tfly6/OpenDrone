"""Non-destructive manuscript audits. No ROS master or flight is started.

The replay uses the current analyzer for extraction; its geometric audit is a
separate sampled-position rule, not a replacement for native mission status.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
import subprocess
import shutil
from pathlib import Path

import numpy as np

from flight_eval.analyzer import BagAnalyzer
from flight_eval.metrics import MetricsCalculator
from .paths import PACKAGE_ROOT, REPOSITORY_ROOT, workspace_root, artifacts_root

POLICY = "sampled-ordered-visitation-v1"


def read_json(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def write_csv(path, rows):
    rows = list(rows)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def ordered_visitation(positions, times, goals, tolerance):
    """3D <= tolerance, sampled points only, zero dwell, ordered goals.

    A sample may satisfy consecutive overlapping goal balls. No interpolation
    through gaps, no estimation accuracy claim, and no terminal-state input.
    """
    positions = np.asarray(positions, dtype=float).reshape((-1, 3))
    goals = np.asarray(goals, dtype=float).reshape((-1, 3))
    times = np.asarray(times, dtype=float)
    if (not len(positions) or not len(goals) or len(times) != len(positions)
            or not np.isfinite(positions).all() or not np.isfinite(goals).all()
            or not np.isfinite(times).all() or np.any(np.diff(times) < 0)):
        return {"complete": None, "visited": 0, "visit_times": [], "reason": "missing_or_invalid_geometry"}
    visited, visit_times = 0, []
    for position, stamp in zip(positions, times):
        while visited < len(goals) and np.linalg.norm(position - goals[visited]) <= tolerance:
            visit_times.append(float(stamp))
            visited += 1
    return {"complete": visited == len(goals), "visited": visited,
            "visit_times": visit_times, "reason": "sampled_ordered_3d"}


def contact_episodes(events, start, end, gap=0.5):
    """Bag-time replay; separate from the historical wall-clock debounce."""
    last, retained = {}, []
    for event in sorted(events, key=lambda e: e["time"]):
        other = event["other_collision"]
        if not start <= event["time"] <= end or not other or "ground_plane" in other:
            continue
        previous = last.get(other)
        last[other] = event["time"]
        if previous is None or event["time"] - previous > gap:
            retained.append(event)
    return retained


def diagnostic_labels(native, geometry, contact, coverage):
    labels = []
    if native == "succeeded" and geometry is False:
        labels.append("native_success_without_geometric_fulfillment")
    if native == "not_succeeded" and geometry is True:
        labels.append("geometric_fulfillment_without_native_success")
    if contact is True:
        labels.append("recorded_contact")
    if coverage is not None and coverage < 0.9:
        labels.append("reference_coverage_below_0.9")
    if geometry is None:
        labels.append("geometric_evidence_unknown")
    if contact is None:
        labels.append("contact_evidence_unknown")
    return labels


def matching_frame(frame, other, aliases):
    def canonical(value):
        value = str(value).lstrip("/")
        return aliases.get(value, value)
    return bool(frame and other) and canonical(frame) == canonical(other)


def inspect_bag(case_dir, case, report, manifest):
    import rosbag
    bag_path = case_dir / "flight_test.bag"
    run = case.get("run_options", {})
    analyzer = BagAnalyzer(str(bag_path), case["controller"]["name"],
                           task_name=case["task"], planner_name=case["planner"]["name"],
                           hover_height=run.get("takeoff_height", 2), duration=run.get("duration"))
    data = analyzer._extract_bag_data()
    phase = analyzer._extract_task_phase(data)
    start, end = float(phase["start_time"]), float(phase["end_time"])
    controller_end = end
    # The first valid native terminal is an observation boundary, not a
    # geometric success condition. Explicit runner ROS boundaries take priority.
    metadata = analyzer.run_metadata
    transitions = metadata.get("lifecycle", {}).get("transitions", [])
    execution = [x for x in transitions if x.get("state") == "executing"
                 and x.get("task_clock") == "ros" and x.get("task_time") is not None]
    if execution:
        start = max(start, float(execution[0]["task_time"]))
    exits = [float(x["task_time"]) for x in transitions
             if x.get("state") in ("requesting_land", "aborting", "capturing_artifacts")
             and x.get("task_clock") == "ros" and x.get("task_time") is not None
             and float(x["task_time"]) >= start]
    end_source = "controller_phase_end"
    if exits:
        end = min(end, min(exits))
        end_source = "runner_ros_exit_and_controller_phase"
    expected_type = 1 if case["task"] == "sequential_goal_mission" else 2
    terminal = [x for x in data["planner_mission_states"]
                if start <= x["message_time"] <= end and x["mission_type"] == expected_type
                and x["status"] in (2, 3) and 0 <= x["completed_items"] <= x["total_items"]
                and x["total_items"] > 0
                and (x["status"] != 2 or x["completed_items"] == x["total_items"])]
    if terminal:
        end = min(end, terminal[0]["message_time"])
        end_source += "+native_terminal_cutoff"
    mask = (data["position_times"] >= start) & (data["position_times"] <= end)
    times, positions = data["position_times"][mask], data["positions"][mask]
    frames = data["position_frames"][mask]
    paths = [p for p in data["mission_paths"] if p["time"] <= end]
    path = paths[0] if paths else {}
    path_changed = any(p["positions"] != path.get("positions") or p["frame_id"] != path.get("frame_id") for p in paths)
    aliases = manifest.get("frame_aliases", {})
    frame_ok = bool(len(frames)) and all(matching_frame(f, path.get("frame_id"), aliases) for f in frames)
    goals = path.get("positions", [])
    result = {"policy": POLICY, "start": start, "end": end,
              "controller_phase_end": controller_end, "end_source": end_source,
              "time_source": "rosbag_record_time; new runner transitions use ROS task clock",
              "frame_aliases": aliases, "geometry_frame_valid": frame_ok,
              "path_changed": path_changed, "goals": goals, "path_frame": path.get("frame_id"),
              "state_frames": sorted(set(str(f) for f in frames)), "state_samples": len(times),
              "geometric_results": {}, "warnings": []}
    result["task_instance_hash"] = hashlib.sha256(json.dumps(
        {"goals": goals, "frame": path.get("frame_id")}, sort_keys=True).encode()).hexdigest()
    for tolerance in manifest["tolerances_m"]:
        audit = ordered_visitation(positions, times, goals, tolerance)
        if not frame_ok or path_changed or not phase.get("execution_started"):
            audit.update(complete=None, reason="frame_path_or_execution_not_validated")
        # Waypoint visitation is not a complete continuous-path oracle.
        if case["task"] == "reference_path_mission":
            audit["waypoint_proxy_complete"] = audit["complete"]
            audit.update(complete=None, reason="continuous_path_fulfillment_requires_separate_oracle")
        result["geometric_results"][str(tolerance)] = audit
    valid_ref = (data["ref_pos_times"] >= start) & (data["ref_pos_times"] <= end)
    ref_times, references = data["ref_pos_times"][valid_ref], data["reference_positions"][valid_ref]
    tracking = MetricsCalculator().compute_position_tracking_metrics(positions, times, references, ref_times)
    availability = MetricsCalculator().compute_reference_availability(times, ref_times)
    result["tracking"] = {m.name: {"value": float(m.value) if np.isfinite(m.value) else None,
                                     "detail": m.detail, "available": m.available}
                          for m in tracking + availability}
    result["alignment_sensitivity"] = []
    positive_dt = np.diff(times)
    positive_dt = positive_dt[positive_dt > 0]
    base_tolerance = max(1e-6, 1e-3 * float(np.median(positive_dt))) if len(positive_dt) else 1e-6
    for factor in (0.1, 1.0, 10.0):
        count, rmse = 0, None
        if len(times) and len(ref_times):
            idx = np.searchsorted(ref_times, times)
            left, right = np.clip(idx-1, 0, len(ref_times)-1), np.clip(idx, 0, len(ref_times)-1)
            nearest = np.where(abs(ref_times[left]-times) <= abs(ref_times[right]-times), left, right)
            keep = (abs(ref_times[nearest]-times) <= factor*base_tolerance)
            keep &= np.isfinite(positions).all(axis=1) & np.isfinite(references[nearest]).all(axis=1)
            count = int(keep.sum())
            if count >= 10:
                rmse = float(np.sqrt(np.mean(np.sum((positions[keep]-references[nearest[keep]])**2, axis=1))))
        result["alignment_sensitivity"].append({"factor": factor, "tolerance_s": factor*base_tolerance,
                                                "retained": count, "rmse_m": rmse})
    contacts, header_deltas, truth = [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        topics = bag.get_type_and_topic_info().topics
        result["bag_topics"] = {k: v.message_count for k, v in topics.items()}
        result["bag_time_range"] = [bag.get_start_time(), bag.get_end_time()]
        for topic, msg, stamp in bag.read_messages(topics=["/flight_eval/contact_pulse", "/gazebo/model_states", "/mavros/local_position/odom", "/flight_eval/ground_truth/odom"]):
            t = stamp.to_sec()
            if topic == "/flight_eval/contact_pulse":
                contacts.append({"time": t, "other_collision": msg.other_collision,
                                 "self_collision": msg.self_collision})
            elif topic == "/mavros/local_position/odom" and start <= t <= end:
                header_deltas.append(t-msg.header.stamp.to_sec())
            elif topic == "/gazebo/model_states" and start <= t <= end and "iris" in msg.name:
                p = msg.pose[msg.name.index("iris")].position
                truth.append((t, [p.x, p.y, p.z]))
            elif topic == "/flight_eval/ground_truth/odom" and start <= t <= end:
                p = msg.pose.pose.position
                truth.append((msg.header.stamp.to_sec(), [p.x, p.y, p.z]))
    gap = float(metadata.get("collision_summary", {}).get("episode_gap", 0.5))
    result["contact_replay"] = {"mission": contact_episodes(contacts, start, end, gap),
                                "post_mission": contact_episodes(contacts, end+1e-9, result["bag_time_range"][1], gap),
                                "debounce_clock": "bag_time", "stream_completeness": "unverified_no_heartbeat"}
    result["runtime_contact_count"] = metadata.get("collision_summary", {}).get("episode_count")
    result["header_minus_bag_summary"] = ({"bag_minus_header_median_s": float(np.median(header_deltas)),
                                           "bag_minus_header_p95_abs_s": float(np.percentile(abs(np.array(header_deltas)), 95))}
                                          if header_deltas else {})
    result["ground_truth"] = {"available": bool(truth), "sample_count": len(truth),
                              "comparison": "not_computed_requires_world_to_local_origin_validation"}
    result["warnings"].extend(["contact replay uses bag time; historical runtime debounce used monotonic time",
                               "zero recorded contacts does not establish contact stream completeness",
                               "MAVROS odometry is not simulator ground truth"])
    return result


def audit(args):
    root = Path(args.workspace).resolve()
    manifest_path = Path(args.manifest).resolve()
    manifest = read_json(manifest_path)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    rows, errors, cache_count = [], [], 0
    import flight_eval.analyzer
    sources = list(Path(__file__).parent.glob("*.py")) + list(Path(flight_eval.analyzer.__file__).parent.glob("*.py"))
    def source_key(path):
        return str(path.relative_to(REPOSITORY_ROOT))
    source_hashes = {source_key(p): digest(p) for p in sources}
    snapshot = output / "source_snapshot"
    snapshot.mkdir(exist_ok=True)
    for source in sources:
        target = snapshot / source_key(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    selected, plan_rows = [], []
    for batch in manifest["batches"]:
        directory = root / "eval_runs/batches" / batch
        if not directory.is_dir():
            raise FileNotFoundError(directory)
        reports = sorted(directory.glob("cases/*/report.json"))
        selected.extend(reports)
        plan_path = directory / "batch_plan.json"
        planned = read_json(plan_path).get("cases", []) if plan_path.exists() else []
        planned_names = {c["case_name"] for c in planned}
        for planned_case in planned:
            case_dir = directory / "cases" / planned_case["case_name"]
            plan_rows.append({"batch": batch, "case_name": planned_case["case_name"],
                              "planned": True, "report_exists": (case_dir / "report.json").is_file(),
                              "bag_exists": (case_dir / "flight_test.bag").is_file(),
                              "resolved_case_exists": (case_dir / "resolved_case.json").is_file()})
        for report_path in reports:
            if report_path.parent.name not in planned_names:
                plan_rows.append({"batch": batch, "case_name": report_path.parent.name,
                                  "planned": False, "report_exists": True,
                                  "bag_exists": (report_path.parent / "flight_test.bag").is_file(),
                                  "resolved_case_exists": (report_path.parent / "resolved_case.json").is_file()})
    write_csv(output / "plan_to_result.csv", plan_rows)
    if args.case:
        selected = [p for p in selected if args.case in str(p)]
        if not selected:
            raise ValueError(f"No trial matched {args.case!r}")
    if args.limit:
        selected = selected[:args.limit]
    for index, report_path in enumerate(selected, 1):
        directory = report_path.parent
        report, case = read_json(report_path), read_json(directory / "resolved_case.json")
        metrics = {m["name"]: m for m in report.get("metrics", [])}
        tracking, coverage = metrics.get("position_tracking_rmse_3d", {}), metrics.get("reference_coverage_ratio", {})
        online = case["task"] in ("sequential_goal_mission", "reference_path_mission")
        n_state = coverage.get("detail", {}).get("state_sample_count")
        n_error = tracking.get("detail", {}).get("sample_count")
        p95 = metrics.get("position_tracking_p95_3d", {})
        runtime_count = metrics.get("collision_episodes", {}).get("value")
        row = {"trial_id": str(directory.relative_to(root)), "controller": case["controller"]["name"],
               "planner": case["planner"]["name"], "task": case["task"], "repetition": case["repetition"],
               "world": case["simulation"]["gazebo"].get("world"), "sensor": case["simulation"]["gazebo"].get("sensor"),
               "run_status": report.get("run_status", {}).get("status"),
               "candidate_included": report.get("run_status", {}).get("status") == "completed" and
                   (online or case["task"] == "hover" or tracking.get("available", False)),
               "exclusion_reason": "" if tracking.get("available") or online or case["task"] == "hover" else tracking.get("detail", {}).get("error", "tracking_unavailable"),
               "native_status": report.get("task_outcome", {}).get("status"),
               "waypoint_count": report.get("task_outcome", {}).get("evidence", {}).get("waypoint_count"),
               "runtime_contact_count": runtime_count, "contact_evidence_quality": "unverified_stream_completeness",
               "reference_coverage": coverage.get("value"), "max_reference_gap_s": metrics.get("max_reference_gap", {}).get("value"),
               "state_samples": n_state, "rmse_samples": n_error,
               "p95_samples": p95.get("detail", {}).get("sample_count"),
               "retained_fraction": n_error/n_state if n_error is not None and n_state else None,
               "rmse_m": tracking.get("value"), "p95_m": metrics.get("position_tracking_p95_3d", {}).get("value"),
               "execution_duration_s": report.get("execution_duration"), "report_sha256": digest(report_path),
               "resolved_case_sha256": digest(directory / "resolved_case.json"), "replay_status": "not_requested"}
        if args.bags:
            cache = output / "trials" / directory.parent.parent.name / (directory.name + ".json")
            try:
                bag = directory / "flight_test.bag"
                fingerprint = {"bag_sha256": digest(bag), "report_sha256": row["report_sha256"],
                               "case_sha256": row["resolved_case_sha256"], "sources": source_hashes,
                               "metadata_sha256": digest(directory / "run_metadata.json"), "manifest_sha256": digest(manifest_path)}
                cached = None
                if cache.exists():
                    try:
                        cached = read_json(cache)
                    except (OSError, json.JSONDecodeError):
                        pass
                if cached and cached.get("fingerprint") == fingerprint:
                    replay = cached["replay"]
                    cache_count += 1
                else:
                    replay = inspect_bag(directory, case, report, manifest)
                    write_json(cache, {"fingerprint": fingerprint, "trial_id": row["trial_id"], "replay": replay})
                row.update(replay_status="ok", audit_start=replay["start"], audit_end=replay["end"],
                           task_instance_hash=replay["task_instance_hash"],
                           bag_mission_contact_count=len(replay["contact_replay"]["mission"]),
                           bag_post_mission_contact_count=len(replay["contact_replay"]["post_mission"]),
                           ground_truth_available=replay["ground_truth"]["available"])
                for tolerance, result in replay["geometric_results"].items():
                    row["geometry_"+tolerance] = result["complete"]
                    row["waypoint_proxy_"+tolerance] = result.get("waypoint_proxy_complete")
                for name in ("position_tracking_rmse_3d", "position_tracking_p95_3d", "reference_coverage_ratio"):
                    row["replay_"+name] = replay["tracking"][name]["value"]
                row["diagnostic_labels_1m"] = ";".join(diagnostic_labels(
                    row["native_status"], row.get("geometry_1.0"),
                    None if runtime_count is None else runtime_count > 0,
                    replay["tracking"]["reference_coverage_ratio"]["value"]))
            except Exception as exc:
                row["replay_status"] = "error"
                errors.append({"trial_id": row["trial_id"], "error": f"{type(exc).__name__}: {exc}"})
        rows.append(row)
        if args.bags and (index % 10 == 0 or index == len(selected)):
            print(f"Audited {index}/{len(selected)}; errors={len(errors)}; cache={cache_count}", flush=True)
    write_csv(output / "trials.csv", rows)
    joint = collections.Counter()
    for row in rows:
        if row["task"] not in ("sequential_goal_mission", "reference_path_mission"):
            continue
        task_group = ("single_goal" if row["waypoint_count"] == 1 else "multi_goal") if row["task"] == "sequential_goal_mission" else "reference_path"
        for tolerance in manifest["tolerances_m"]:
            geometry = row.get("geometry_"+str(tolerance))
            contact = row["runtime_contact_count"]
            joint[(task_group, row["planner"], tolerance, row["native_status"],
                   "unknown" if geometry is None else str(geometry),
                   "unknown" if contact is None else ("recorded" if contact > 0 else "none_recorded"))] += 1
    write_csv(output / "joint_outcomes.csv", [dict(zip(
        ["task_group", "planner", "tolerance_m", "native_status", "geometric_completion", "runtime_recorded_contact"], key), count=count)
        for key, count in sorted(joint.items())])
    summaries = []
    grouped = collections.defaultdict(list)
    for row in rows:
        grouped[(row["task"], row["controller"], row["planner"])].append(row)
    for key, group in sorted(grouped.items()):
        for threshold in (0.0, 0.8, 0.9, 0.95):
            retained = [r for r in group if r["rmse_m"] is not None and r["reference_coverage"] is not None
                        and r["reference_coverage"] >= threshold]
            summaries.append(dict(zip(["task", "controller", "planner"], key), coverage_threshold=threshold,
                                  trial_count=len(group), retained_trials=len(retained),
                                  median_rmse_m=float(np.median([r["rmse_m"] for r in retained])) if retained else None,
                                  state_samples=sum(r["state_samples"] or 0 for r in group),
                                  retained_error_samples=sum(r["rmse_samples"] or 0 for r in retained)))
    write_csv(output / "coverage_sensitivity.csv", summaries)
    online_rows = [r for r in rows if r["task"] in ("sequential_goal_mission", "reference_path_mission")]
    mismatch = [r["trial_id"] for r in online_rows if r["native_status"] == "succeeded" and r.get("geometry_1.0") is False]
    summary = {"policy": POLICY, "report_count": len(rows), "candidate_included": sum(r["candidate_included"] for r in rows),
               "planned_trials_missing_reports": sum(r["planned"] and not r["report_exists"] for r in plan_rows),
               "reports_without_plan_entry": sum(not r["planned"] for r in plan_rows),
               "bag_replayed_trials": sum(r["replay_status"] == "ok" for r in rows),
               "online_count": len(online_rows), "native_success": sum(r["native_status"] == "succeeded" for r in online_rows),
               "native_success_no_recorded_contact": sum(r["native_status"] == "succeeded" and r["runtime_contact_count"] == 0 for r in online_rows),
               "native_success_geometry_incomplete_1m": mismatch if args.bags else None,
               "of_these_no_recorded_contact": [r["trial_id"] for r in online_rows if r["trial_id"] in mismatch and r["runtime_contact_count"] == 0] if args.bags else None,
               "errors": errors, "cached_trials": cache_count,
               "notes": ["candidate corpus, not a recovered versioned publication release",
                         "continuous path fulfillment is unknown; waypoint proxy is separate",
                         "coverage sensitivity uses archived report metrics; replay metrics are separate",
                         "recorded-contact absence does not certify stream completeness"]}
    write_json(output / "summary.json", summary)
    git = subprocess.run(["git", "-C", str(REPOSITORY_ROOT), "rev-parse", "HEAD"], capture_output=True, text=True)
    diff = subprocess.run(["git", "-C", str(REPOSITORY_ROOT), "diff", "--binary"], capture_output=True, text=True)
    (output / "working_tree.patch").write_text(diff.stdout)
    write_json(output / "provenance.json", {"manifest": manifest, "manifest_sha256": digest(manifest_path),
                                           "git_head": git.stdout.strip(), "source_sha256": source_hashes,
                                           "selection_count": len(selected), "bag_replay": args.bags})
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if errors:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(workspace_root()))
    parser.add_argument("--manifest", default=str(PACKAGE_ROOT / "revision_manifest.json"))
    parser.add_argument("--output", default=str(artifacts_root() / "revision_audit"))
    parser.add_argument("--bags", action="store_true", help="Replay bags without modifying archived reports")
    parser.add_argument("--case", help="Only trials whose paths contain this text")
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    if args.limit is not None and args.limit < 1:
        parser.error("--limit must be positive")
    audit(args)


if __name__ == "__main__":
    main()
