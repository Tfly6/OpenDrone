"""Evaluation protocol checks, mission audits and fixed-task comparisons."""
import argparse
import json
import os
from pathlib import Path
import random
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

import yaml

from flight_eval.batch import FlightEvalBatchRunner, load_batch_definition
from flight_eval.cli import _postprocess_completed_run
from flight_eval.environment import prepare_environment
from flight_eval.launch_inspection import inspect_launch
from flight_eval.planners import get_planner_launch
from revision_support.audit import digest, write_json
from revision_support.runtime import configure_revision_runtime
from revision_support.paths import PACKAGE_ROOT, REPOSITORY_ROOT, SCRIPTS_ROOT, artifacts_root, workspace_root, px4_source

ROOT = workspace_root()
SCRIPTS = SCRIPTS_ROOT
ARTIFACTS = artifacts_root()
WORLDS = {"cylinders": "cylinders_85_45", "forest": "forest"}
configure_revision_runtime()


def check():
    commands = [
        [sys.executable, "-m", "unittest"] + ["checks." + name for name in (
            "test_metrics", "test_environment", "test_batch", "test_planners", "test_visualizer", "test_portability")] + ["revision_support.test_revision"],
        [sys.executable, "-m", "checks.test_mission_path_contracts"],
        [sys.executable, "-m", "checks.test_planner_adapter_evaluators"],
        [sys.executable, "-m", "revision_support.validation", "--output", str(ARTIFACTS / "revision_validation")],
    ]
    for command in commands:
        subprocess.run(command, check=True, cwd=ROOT)
    for world in WORLDS:
        definition, _ = design(world, 5, 20261115, ARTIFACTS / "revision_plans" / world)
        validate_launches(definition)


def design(world, repeat, order_seed, directory, px4_dir=None):
    directory.mkdir(parents=True, exist_ok=True)
    # Archive experiment-specific tools with each new design.
    support = PACKAGE_ROOT / "revision_support"
    shutil.copytree(support, directory / "support_snapshot", dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    tasks = {}
    for name in ("single", "sequential"):
        source = PACKAGE_ROOT / "experiments" / ("task_" + name + ".yaml")
        dest = directory / source.name
        shutil.copy2(source, dest)
        tasks[name] = dest
    config = {
        "version": 1, "name": "revision_" + world,
        "defaults": {"simulation": {
            "px4": {"source_dir": str(Path(px4_dir).expanduser().resolve() if px4_dir else px4_source()), "sim_model": "gazebo-classic_iris"},
            "transport": {"gazebo_px4": "udp", "port": 14560,
                          "environment_dir": str(directory / "environment")},
            "mavros": {"fcu_url": "udp://:14540@localhost:14557", "respawn_mavros": True},
            "gazebo": {"world": WORLDS[world], "sensor": "depth_camera", "gui": False,
                       "respawn_gazebo": True, "x": 0, "y": 0, "z": 0}},
            "run": {"takeoff_height": 2.0, "duration": 120, "auto_land": True,
                    "max_collision_episodes": 5, "collision_episode_gap": 0.5,
                    "extra_topics": ["/flight_eval/ground_truth/odom", "/flight_eval/ground_truth/query_duration",
                                     "/clock", "/tf", "/tf_static"]}},
        "experiments": [{"name": "matched_" + name, "task": "sequential_goal_mission",
                          "controllers": ["se3_hopf", "pid_controller"],
                          "planners": ["ego_planner", "fast_planner_kino"], "repeat": repeat,
                          "planner_args": {"waypoint_type": "manual", "waypoint_config": str(path),
                                           "start_from_current_position": "false",
                                           "publish_horizon_reference": "true", "use_rviz": "false"}}
                         for name, path in tasks.items()],
    }
    config_path = directory / "batch.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    batch_output = ROOT / "eval_runs/batches" / ("revision_" + world + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    definition = load_batch_definition(str(config_path), output_dir=str(batch_output))
    random.Random(order_seed).shuffle(definition.cases)
    # Freeze the order with original repetition identities and case names.
    write_json(directory / "design.json", {
        "world": world, "repeat": repeat, "execution_order_seed": order_seed,
        "seed_scope": "execution-order only; repetitions are not independent random map seeds",
        "task_sha256": {name: digest(path) for name, path in tasks.items()},
        "config_sha256": digest(config_path), "ordered_cases": [c.to_dict() for c in definition.cases],
        "scope": "same task geometry and execution budget; native planner dynamic limits retained",
        "limitations": ["Iris/PX4/Gazebo only; no second vehicle or real-flight validation",
                        "SDF endpoint screening is not a proof of collision-free reachability",
                        "native planner thresholds and dynamic constraints must be interpreted separately"]})
    print(f"{world}: {len(definition.cases)} trials, design: {directory}", flush=True)
    return definition, directory


def validate_launches(definition):
    seen = set()
    for case in definition.cases:
        requested = dict(case.planner["args"])
        key = (case.planner["name"], json.dumps(requested, sort_keys=True))
        if key in seen:
            continue
        seen.add(key)
        planner = get_planner_launch(case.planner["name"], requested)
        args = dict(planner["args"])
        args.update(auto_trigger_waypoints="true", use_preset_waypoints="true")
        inspected = inspect_launch(planner["launch_pkg"], planner["launch_file"], args,
                                   resolved_path=str(SCRIPTS.parent / "launch" / planner["launch_file"]))
        # Resolve actual XML/YAML values rather than merely checking arg names.
        import roslaunch.config
        cfg = roslaunch.config.load_config_default(
            [(inspected["resolved_path"], [f"{k}:={v}" for k, v in args.items()])],
            port=None, verbose=False, assign_machines=False)
        task = yaml.safe_load(Path(requested["waypoint_config"]).read_text())
        for name, expected in task.items():
            actual = cfg.params["/waypoint_generator/" + name].value
            if actual != expected:
                raise ValueError(f"Task parameter mismatch: {name}: {actual!r} != {expected!r}")
        if cfg.params["/waypoint_generator/startFromCurrentPosition"].value is not False:
            raise ValueError("Matched tasks must use absolute task coordinates")
    print(f"Validated {len(seen)} planner/task launch combinations", flush=True)


def wait_environment(process, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("Gazebo/MAVROS exited; inspect environment.log")
        probe = subprocess.run(["rosservice", "list"], capture_output=True, text=True, timeout=10)
        if "/gazebo/get_model_state" in probe.stdout and "/mavros/set_mode" in probe.stdout:
            model = subprocess.run(["rosservice", "call", "/gazebo/get_model_state", "model_name: 'iris'"],
                                   capture_output=True, text=True, timeout=10)
            if "success: True" in model.stdout or "success: true" in model.stdout:
                return
        time.sleep(1)
    raise TimeoutError("Gazebo/MAVROS not ready after 120s; inspect environment.log")


def run_sitl(args):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    directory = ARTIFACTS / "revision_runs" / (args.world + "_" + stamp)
    definition, directory = design(args.world, args.repeat, args.order_seed, directory, args.px4_source)
    validate_launches(definition)
    execute_definition(definition, directory)


def execute_definition(definition, directory):
    """Run a prepared single-environment batch and archive its evidence."""
    existing = subprocess.run(["rosnode", "list"], capture_output=True, text=True, timeout=10)
    nodes = existing.stdout.splitlines()
    if any(n == "/gazebo" or n.startswith("/mavros") for n in nodes):
        raise RuntimeError("Gazebo/MAVROS already running; stop the existing simulation before starting this isolated batch")
    prepared = prepare_environment(definition)
    # Read only the generated environment through bash; preserve environment
    # values including spaces, and avoid eval/command-string interpolation.
    env_result = subprocess.run(["bash", "-c", 'source "$1"; env -0', "revision-env", prepared["shell_file"]],
                                capture_output=True, check=True)
    env = dict(item.split("=", 1) for item in env_result.stdout.decode().split("\0") if "=" in item)
    previous_env = os.environ.copy()
    os.environ.update(env)
    log = (directory / "environment.log").open("w")
    process = None
    truth_process = None
    try:
        process = subprocess.Popen(["bash", prepared["launch_command_file"]], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f"Starting isolated simulation; log: {directory / 'environment.log'}", flush=True)
        wait_environment(process)
        truth_process = subprocess.Popen([sys.executable, str(PACKAGE_ROOT / "record_ground_truth.py")],
                                         env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        ready = subprocess.run(["rostopic", "echo", "-n", "1", "/flight_eval/ground_truth/odom"],
                               capture_output=True, text=True, timeout=30)
        if ready.returncode or truth_process.poll() is not None:
            raise RuntimeError("Simulator ground-truth recording did not become ready")
        result_dir = Path(definition.output_dir)
        print(f"Running batch: {result_dir}", flush=True)
        summary = FlightEvalBatchRunner(definition, continue_on_error=True,
                                       result_handler=_postprocess_completed_run).run()
        write_json(directory / "completed_batch.json", {"output_dir": str(result_dir), "summary_file": summary["summary_file"]})
        # Keep the exact task files and randomized execution plan with the run.
        shutil.copytree(directory, result_dir / "revision_design", ignore=shutil.ignore_patterns("environment", "environment.log"))
        manifest = {"batches": [result_dir.name], "tolerances_m": [0.5, 1.0, 2.0],
                    "frame_aliases": {"world": "map"}}
        write_json(directory / "audit_manifest.json", manifest)
        subprocess.run([sys.executable, "-m", "revision_support.audit", "--bags",
                        "--workspace", str(ROOT),
                        "--manifest", str(directory / "audit_manifest.json"),
                        "--output", str(directory / "audit")], check=True)
        if (len(summary["cases"]) != len(definition.cases) or
                any(c["status"] != "completed" or c.get("analysis_error") for c in summary["cases"])):
            raise RuntimeError(f"Incomplete execution/analysis recorded in {summary['summary_file']}")
    finally:
        if truth_process is not None and truth_process.poll() is None:
            os.killpg(truth_process.pid, signal.SIGINT)
            try:
                truth_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(truth_process.pid, signal.SIGKILL)
                truth_process.wait()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()
        os.environ.clear()
        os.environ.update(previous_env)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="Tests, synthetic validation and ROS launch parameter checks; no simulator")
    audit_parser = sub.add_parser("audit", help="Audit archived candidate corpus")
    audit_parser.add_argument("--bags", action="store_true")
    audit_parser.add_argument("--case")
    audit_parser.add_argument("--limit", type=int)
    audit_parser.add_argument("--output", default=str(ARTIFACTS / "revision_audit"))
    review_parser = sub.add_parser("review", help="Inspect completed revision runs and simulator truth")
    review_parser.add_argument("--output", default=str(ARTIFACTS / "revision_review_20261116"))
    for name in ("plan", "sitl"):
        p = sub.add_parser(name)
        p.add_argument("world", choices=list(WORLDS))
        p.add_argument("--repeat", type=int, default=5)
        p.add_argument("--order-seed", type=int, default=20261115)
        p.add_argument("--px4-source", default=None, help="PX4 source directory; default PX4_SOURCE_DIR or ~/PX4-Autopilot")
    args = parser.parse_args()
    if getattr(args, "repeat", 1) < 1:
        parser.error("--repeat must be positive")
    if args.command == "check":
        check()
    elif args.command == "review":
        from revision_support.review import review
        review(ROOT, args.output)
    elif args.command == "audit":
        command = [sys.executable, "-m", "revision_support.audit", "--workspace", str(ROOT),
                   "--manifest", str(PACKAGE_ROOT / "revision_manifest.json"), "--output", args.output]
        if args.bags:
            command.append("--bags")
        if args.case:
            command.extend(["--case", args.case])
        if args.limit:
            command.extend(["--limit", str(args.limit)])
        subprocess.run(command, check=True)
    elif args.command == "plan":
        definition, _ = design(args.world, args.repeat, args.order_seed,
                               ARTIFACTS / "revision_plans" / args.world, args.px4_source)
        validate_launches(definition)
    elif args.command == "sitl":
        run_sitl(args)


if __name__ == "__main__":
    def interrupted(signum, frame):
        del signum, frame
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main()
    except KeyboardInterrupt:
        print("Revision workflow interrupted; owned simulation processes cleaned up.", file=sys.stderr)
        raise SystemExit(130)
