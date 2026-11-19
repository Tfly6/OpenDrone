# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project at a glance

OpenDrone is a ROS1 + PX4 SITL research workspace for evaluating quadrotor controllers, trajectory generators, and online planners against the same simulator. The active evaluation surface is the `opendrone` catkin package; controllers and planners live as separate catkin packages alongside shared utility packages. The standard work cell is Gazebo Classic SITL + MAVROS + PX4 + a controller ROS node + (optionally) a planner ROS node.

Default repository context: `src/OpenDrone`. On the `dev` branch, source code and launch files outrank README/handoff descriptions; if they disagree, say so rather than silently following the docs.

**Do not rely on the top-level `src/OpenDrone/README.md`.** It is a frozen early-development intro (ROS Melodic/Noetic setup, dependency install, "offb_node takeoff" demo) and has not been updated for the current `flight_eval` benchmark, the unified `/planner/output` contract, or any of the `controller/*` / `planner/*` packages. Treat it as historical context only. Trust the source, launch files, `package.xml`/`CMakeLists.txt`, `opendrone/scripts/flight_eval/README.md`, and the handoff notes over `README.md`.

## Common commands

### Build (catkin workspace root, usually `~/catkin_ws`)

```bash
catkin build opendrone             # main package only
catkin build                       # full workspace, including controllers/planners/utils
source devel/setup.bash            # overlay every terminal that runs ROS nodes
```

`opendrone` depends on `geometry_msgs mavros_msgs message_runtime roscpp rospy sensor_msgs std_msgs std_srvs trajectory_msgs tf2_ros tf2_sensor_msgs pcl_ros` and Eigen3. System deps for the broader workspace: `libgoogle-glog-dev libgflags-dev libeigen3-dev libarmadillo-dev` plus `ros-<distro>-ompl`. NLopt (v2.7.1) must be installed from source before building the workspace.

### Run a single SITL demo (PX4 + MAVROS + one of the basic examples)

```bash
# Terminal 1: PX4 SITL in Gazebo
roslaunch px4 mavros_posix_sitl.launch

# Terminal 2: built-in offboard example
source <workspace>/devel/setup.bash
rosrun opendrone offb_node        # takeoff to 2 m and hold
rosrun opendrone base             # state machine consuming /planner/output
```

`base` is the canonical reference consumer of `/planner/output`; all custom controllers and the `flight_eval` benchmark are expected to follow the same flight state codes it publishes.

### flight_eval CLI (single run / batch / analysis / visualization)

The package is imported as `flight_eval`; PYTHONPATH must include `src/OpenDrone/opendrone/scripts` (no `setup.py` install). Install the small Python extras listed in `opendrone/scripts/flight_eval/requirements.txt` (`numpy`, `matplotlib`, `Pillow`, `PyYAML`).

```bash
cd <workspace_root>
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH

python3 -m flight_eval list-controllers
python3 -m flight_eval list-tasks
python3 -m flight_eval list-planners

# single run
python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_circle \
  --planner rpg_trajectory

# analysis only (no SITL)
python3 -m flight_eval analyze \
  --bag eval_runs/<run_dir>/flight_test.bag \
  --controller se3_hopf \
  --task sequential_goal_mission \
  --planner ego_planner_v2

# re-render trajectory plots / GIF
python3 -m flight_eval visualize \
  --bag eval_runs/<run_dir>/flight_test.bag \
  --controller se3_hopf --task sequential_goal_mission \
  --planner fast_planner_kino \
  --gif --gif-fps 15 --gif-duration 12
```

The detailed run/analyze/batch/visualize reference lives in `src/OpenDrone/opendrone/scripts/flight_eval/README.md` — read it before adding a new controller, planner, or task; the contract there is what the runner, analyzer, and external manifests all rely on.

### flight_eval batch (YAML-driven; uses UDP PX4 transport)

A batch never spawns Gazebo/MAVROS itself — those are started by the user once with `prepare-environment`. Each case owns its own PX4 SITL process and rootfs.

```bash
python3 -m flight_eval prepare-environment --config <yaml>     # one-time per PX4/world/sensor
source /tmp/flight_eval_udp_environment/environment.env
bash /tmp/flight_eval_udp_environment/launch_command.sh         # keep this running

python3 -m flight_eval batch --config <yaml> --dry-run         # validate expansion first
python3 -m flight_eval batch --config <yaml>                   # actually run
python3 -m flight_eval batch --config <yaml> --continue-on-error --no-analyze
```

YAML schema is documented in `opendrone/scripts/flight_eval/examples/batch_example.yaml` (and `examples/batch_example_missions.yaml` for integrated missions like `sequential_goal_mission` and `reference_path_mission`). The transport must stay constant across the whole batch; switching world/sensor/transport requires a new YAML and a fresh `prepare-environment`.

### flight_eval unit tests

```bash
python3 -m compileall -q opendrone/scripts/flight_eval
PYTHONPATH=opendrone/scripts python3 -m unittest discover \
  -s opendrone/scripts/flight_eval/tests -v
```

The standalone numerical test for `planner_adapter.EgoV2PolyTrajEvaluator`:

```bash
cd src/OpenDrone/opendrone/scripts
python3 test_planner_adapter_evaluators.py
```

Note: `opendrone/scripts/flight_eval/tests/` is matched by the generic `tests/` rule in the workspace `.gitignore` and will not appear in plain `git status`. New tests in that directory exist locally but won't be committed until the ignore rule is changed.

### Paper figures (post-hoc batch analysis)

```bash
source /opt/ros/noetic/setup.bash
export PYTHONPATH="$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH"
python3 -m flight_eval.paper_figures \
  --workspace-root "$PWD" --figures all
```

Batch selection and priority live in `opendrone/scripts/flight_eval/paper_figures/figure_manifest.yaml` — update there, not in the generated CSVs.

## High-level architecture

### Repository layout

```
src/OpenDrone/
├── opendrone/                     # main catkin package (the project entrypoint)
│   ├── CMakeLists.txt             # builds all executable nodes + catkin_python scripts
│   ├── msg/PlannerOutput{Point}.msg
│   ├── include/opendrone/         # planner_output_utils.h, printf_utils.h
│   ├── src/
│   │   ├── basic_example/         # offb_node, off_node*, rect*, waypoint — minimal examples
│   │   ├── base.cpp               # reference consumer of /planner/output (state machine)
│   │   ├── depth_camera_processor.cpp, trigger_land.cpp, print_info.cpp, test.cpp
│   ├── launch/
│   │   ├── sitl_*.launch          # per-algorithm SITL wiring (controllers + planners)
│   │   ├── benchmarks/            # benchmark launch wrappers used by flight_eval
│   │   │   ├── controller_benchmark/
│   │   │   ├── trajectory_generation_benchmark/
│   │   │   └── online_planner_benchmark/
│   │   └── flight_eval/environment.launch  # Gazebo+MAVROS-only environment for batch
│   ├── sitl_config/               # git submodule — Gazebo worlds, models, plugins
│   ├── scripts/
│   │   ├── planner_adapter.py     # algorithm-native msgs → opendrone/PlannerOutput
│   │   ├── analytic_reference_publisher.py
│   │   ├── camera_pose_publisher.py, lidar_pose_publisher.py
│   │   ├── trajectory_msg_converter.py, waypoint_msg_converter.py
│   │   ├── pointcloud_to_world.py
│   │   ├── test_px4_reset.py, test_planner_adapter_evaluators.py
│   │   └── flight_eval/           # YAML batch runner + analyzer + visualizer (see below)
│   └── rviz/                      # saved RViz configs
├── controller/                    # catkin packages per controller family
│   ├── geometric_controller/{se3_hopf, se3_lee}
│   ├── lqr_controller/
│   ├── mpc_controller/{mav_linear_mpc, mav_nonlinear_mpc, mav_disturbance_observer}
│   └── pid_controller/
├── planner/                       # catkin packages per planner family
│   ├── fast_planner/              # HKUST Fast-Planner (kino + topo)
│   ├── ego_planner_swarm/         # EGO-Planner (depth camera)
│   ├── ego_planner_v2/            # EGO-Planner v2 (PolyTraj native)
│   ├── airfar_planner/            # UPenn Air-FAR
│   ├── super_planner/             # HKUST SUPER planner + rog_map
│   ├── mav_trajectory_generation/ # ETH polynomial trajectory generator
│   ├── rpg_polynomial_trajectory/ # RPG polynomial trajectory generator
│   └── spline_local_planner/
├── utils/                         # shared third-party catkin packages (mostly git submodules)
│   ├── catkin_simple/, mav_comm/, quadrotor_msgs/
│   ├── math_utils/, odom_visualization/, rviz_plugins/
│   ├── multi_map_server/, drone_detect/, waypoint_generator/
│   ├── airfar_reference_bridge/, opendrone_gazebo_plugins/
│   └── mav_comm/{mav_msgs, mav_planning_msgs, mav_state_machine_msgs, mav_system_msgs, mav_comm}
├── cmake/Modules/                 # additional CMake finders
├── shell/                         # dev helpers: gazeboSetup.bash, record*.sh, trigger_land.sh
├── handoff.md, handoff1.md, handoff2.md, summary.md, research.md
└── README.md, LICENSE
```

### Unified control contract

All controllers and planners in this repo converge on two public interfaces so the benchmark can stay algorithm-agnostic:

- `/flight_state` (`std_msgs/Int8`) — published by the controller. State codes are the canonical enum from `opendrone/src/base.cpp`: `0 WAITING_FOR_CONNECTED`, `1 WAITING_FOR_OFFBOARD`, `2 TAKEOFF`, `3 MISSION_EXECUTION`, `4 LANDING`, `5 LANDED`, `6 EMERGENCY`. The benchmark runner treats these as the *input* to its own higher-level lifecycle state machine.
- `/planner/output` (`opendrone/PlannerOutput`) — published by the planner (or by `planner_adapter.py`). Defined in `opendrone/msg/PlannerOutput.msg` and `PlannerOutputPoint.msg`. Each message carries `trajectory_id`, `is_horizon`, `trajectory_start_time`, and an array of `PlannerOutputPoint`s. Per-point fields (position, velocity, acceleration, jerk, snap, yaw, yaw_rate, angular_velocity) are gated by `valid_mask`; position is required, everything else is optional and consumers must ignore unset bits. Invariant: `absolute_desired_time = trajectory_start_time + point.time_from_start`. The `header.frame_id` is the ROS ENU frame the planner reasons in (typically `map`; some planners use `world` and rely on a zero-offset static TF).

The C++ helper `opendrone::planner_output::Select*` functions in `opendrone/include/opendrone/planner_output_utils.h` centralize the `valid_mask` handling for controllers.

Reference consumer for this contract: `opendrone/src/base.cpp` — a position-baseline state machine that requests OFFBOARD, takes off to `takeoff_height`, then follows the first valid `position` (and optional yaw) of each `/planner/output`. The minimal PX4 offboard examples under `opendrone/src/basic_example/` predate the unified contract and bypass it.

### flight_eval architecture

`opendrone/scripts/flight_eval/` is a self-contained Python package that drives the whole benchmark pipeline:

- `cli.py` / `__main__.py` — argparse entry point with subcommands `run / batch / prepare-environment / analyze / visualize / list-controllers / list-tasks / list-planners`.
- `runner.py` (`FlightRunner`) — orchestrates one case: state machine `PREPARING → RECORDING → WAITING_FOR_CONTROLLER → STARTING_PLANNER → WAITING_FOR_PLANNER → EXECUTING → REQUESTING_LAND → WAITING_FOR_LANDED → CAPTURING_ARTIFACTS → COMPLETED` (plus `ABORTING / FAILED / INTERRUPTED`). All spawned `rosbag`/`roslaunch`/PX4 processes are tracked in registered process groups so SIGINT → SIGTERM → SIGKILL escalation can clean up even if the group leader has already died (`process_cleanup` is recorded in `run_metadata.json`; `SIGKILL` survivors flip `run_status` to `failed / process_cleanup_incomplete`). `task_clock_stalled` aborts a case when ROS time stops advancing for `TASK_CLOCK_STALL_TIMEOUT = 30.0` s.
- `analyzer.py` (`BagAnalyzer`) — reads a recorded bag and emits `report.json` + `agent_summary.md`. Slices phases from `/flight_state`, computes metrics with `metrics.MetricsCalculator`, and now separates `run_status` (process lifecycle) from `task_outcome` (semantic success/failure).
- `tasks.py` / `outcomes.py` — task-specific evaluators. Tasks (`hover`, `analytic_circle`, `analytic_figure8`, `analytic_spiral`, `discrete_circle`, `discrete_figure8`, `sequential_goal_mission`, `reference_path_mission`) define `metric_profile`, `execution_start_policy`, optional terminal outcome (only `discrete*`, `sequential_goal_mission` and `reference_path_mission` are finite), and which waypoint/trigger topics they consume. Finite tasks consume `/waypoint_generator/waypoints` (`nav_msgs/Path`); analytic tasks use `analytic_reference_publisher.py`.
- `metrics.py` — pure-numpy metric calculators; deliberately returns facts, not scores. The report groups them (`outcome / tracking / spatial_fidelity / trajectory_quality / stability / interface_health / diagnostic`) and never aggregates into a single rating.
- `controllers.py` / `planners.py` — registries for built-in algorithms. Both list a `kind`, launch package/file, default `args`, optional `tasks` whitelist, `record_topics`, and any `runtime_parameter_expectations` that must be validated before launch.
- `batch.py` (`FlightEvalBatchRunner`) — YAML-driven multi-case runner. Loads the YAML, validates it, builds a `_SimulationSession` per case, owns the connection-lifecycle subscriber, and applies the same SIGINT/SIGTERM upgrade rules as `FlightRunner`. The YAML schema is enforced by `_ROOT_KEYS / _DEFAULT_KEYS / _EXPERIMENT_KEYS / _SIMULATION_KEYS / _TRANSPORT_KEYS` constants in this file — extend those when adding fields.
- `environment.py` — `prepare-environment` logic: copies the Iris model from the PX4 source pointed at by `simulation.px4.source_dir`, rewrites `mavlink_interface/use_tcp=0` and `mavlink_udp_port=<port>`, injects a lightweight `opendrone_contact` plugin, and writes the user-run `environment.launch` and `launch_command.sh`. Does **not** modify the PX4 source or build directory.
- `manifests.py` + `schemas/algorithm_manifest.schema.yaml` — external algorithm manifest loader. New controllers/planners are added by shipping a YAML manifest that conforms to the schema; `flight_eval` merges it into the controller/planner registry at runtime via `--manifest` (or the batch YAML's `manifests:` list).
- `controllers.py` / `planners.py` — the "manifest or built-in" indirection point; every new algorithm must also satisfy the standard interfaces in `_STANDARD_INTERFACES` (`{'controller': {'flight_state': '/flight_state'}, 'planner': {'planner_output': '/planner/output'}}`).

Adding a new algorithm means: (a) ship a benchmark launch under `opendrone/launch/benchmarks/...` that publishes the right state/`PlannerOutput`; (b) either register it in `controllers.py` / `planners.py` or provide a manifest YAML; (c) make sure `record_topics` covers any diagnostic topics you need to see in offline analysis.

### Algorithm-native → PlannerOutput

Planners in this repo don't speak `PlannerOutput` natively; `opendrone/scripts/planner_adapter.py` is the bridge. Supported adapter modes:

- `bspline` — uniform B-spline (`quadrotor_msgs/Bspline`) De Boor evaluation, derivatives, optional horizon sampling on a timer (`publish_horizon_on_timer`).
- `position_command` — single-point `quadrotor_msgs/PositionCommand` republished as the latest reference.
- `ego_v2_poly_traj` — `quadrotor_msgs/PolyTraj` piecewise polynomial evaluator. Coefficients are stored by descending power `[c5, c4, c3, c2, c1, c0]`; the wire order is **not reversed**. `test_planner_adapter_evaluators.py` numerically pins this contract.

Each `sitl_<planner>.launch` chooses the adapter by setting `adapter_type` and `input_topic` on the `planner_adapter` node and forward-declaring the right controller.

### PX4 / Gazebo wiring

- `opendrone/launch/flight_eval/environment.launch` is the only launch file intended to outlive a single case. It starts Gazebo Classic, spawns the vehicle, and MAVROS — all three are configured `respawn=true` so PX4 restarts don't tear them down.
- `opendrone/sitl_config/` is a git submodule (branch `ros1`); it owns worlds, models, and Gazebo plugins. `generate_cylinder_world.py` under it programmatically generates `opendrone/sitl_config/worlds/...` cylinder worlds.
- `opendrone/scripts/pointcloud_to_world.py` rotates camera-frame point clouds into the planner frame for online replanners.
- `shell/gazeboSetup.bash` is a thin wrapper that pre-sources the right ROS/MAVROS environment before launching `roslaunch px4 mavros_posix_sitl.launch`.

### Coordinate-frame convention

`/mavros/local_position/odom` and `/waypoint_generator/waypoints` share one ROS ENU frame (typically `map`). Some planners reason internally in `world`; a zero-offset `tf2 static_transform_publisher world map` (`world_map_linker`) is the conventional bridge, and analyzers compare the two `header.frame_id`s explicitly. A mismatch is recorded as `task_outcome=unknown / frame_mismatch` rather than guessed.

### Submodule + third-party layout

- `opendrone/sitl_config`, `utils/catkin_simple`, `utils/mav_comm` are git submodules (see `.gitmodules`).
- Algorithm packages under `controller/` and `planner/` are usually vendored forks of upstream research code (ETH `mav_control_rw`, Jaeyoung-Lim `mavros_controllers`, HKUST Fast-Planner / EGO-Planner / SUPER, UPenn Air-FAR, RPG quadrotor control). When behavior or interfaces are unclear, cross-check against the upstream project before assuming the OpenDrone implementation is standalone. The mapping is documented in `summary.md` and `research.md`.

## Useful diagnostic topics during a run

```text
/mavros/local_position/odom
/mavros/state
/mavros/local_position/pose
/mavros/setpoint_position/local          # px4_position_baseline output
/mavros/setpoint_raw/attitude            # se3_* / mpc attitude + thrust
/mavros/setpoint_raw/local               # pid attitude (loose coupling)
/command/roll_pitch_yawrate_thrust       # mpc body-rate + thrust
/command/trajectory                      # legacy MultiDOFJointTrajectory reference for some controllers
/planner/output                          # unified PlannerOutput reference
/flight_state                            # controller state code
/waypoint_generator/waypoints            # nav_msgs/Path consumed by finite tasks
/waypoint_generator/traj_start_trigger   # discrete*/sequential_goal_mission start signal
/flight_eval/contact_pulse               # Gazebo contact events emitted by opendrone_contact plugin
```

## Pointers before you start editing

- `handoff.md` — the most recent design handoff; read it before changing anything in `flight_eval/`, `planner_adapter.py`, or the SITL launch files. It also describes the SE3 Hopf + RPG discrete-circle case that motivated the current `PathGoalEvaluator`.
- `opendrone/scripts/flight_eval/README.md` — the contract for everything benchmark-related (topics, messages, allowed task/planner/controller names, YAML schema, output artifacts).
- `opendrone/scripts/flight_eval/schemas/algorithm_manifest.schema.yaml` — the schema to follow when adding an external algorithm manifest.
- `opendrone/launch/benchmarks/README.md` — the benchmark launch layout convention.
- `AGENTS.md` at the workspace root — overrides default assumptions about scope and gives the rule "source outranks README on the `dev` branch."
- The top-level `README.md` is **not** a current reference — see the note at the top of this file.