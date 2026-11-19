[English](developer-guide.md) | [简体中文](developer-guide_CN.md)

# Flight Evaluation Developer Guide

This document defines the internal model and integration contracts of `flight_eval`. Operational instructions belong in the [user guide](user-guide.md).

## Conceptual model

```text
FlightRunner / BagAnalyzer
  |-- task: experiment semantics and metric profile
  |-- controller: vehicle-control launch and diagnostic topics
  |-- planner: reference or planning launch and diagnostic topics
  |-- run_status: execution and artifact-capture result
  `-- task_outcome: semantic completion result
```

A task owns the meaning of an experiment: its phases, start policy, duration or deadline, protocol topics, outcome evaluator, and complete metric set. Controller and planner entries describe how an algorithm is launched and which algorithm-specific topics should be recorded.

Do not infer task success from process success. A run can capture all artifacts successfully while the vehicle fails to complete the task.

## Source layout

| File | Responsibility |
|---|---|
| `cli.py` | Commands, argument parsing, postprocessing, and artifact entry points |
| `runner.py` | Process orchestration, flight lifecycle, recording, and cleanup |
| `batch.py` | YAML validation, matrix expansion, environment sessions, and per-case PX4 |
| `environment.py` | External Gazebo/MAVROS environment generation |
| `tasks.py` | Task definitions, phases, outcome evaluators, and metric composition |
| `controllers.py` | Built-in controller registry |
| `planners.py` | Built-in planner registry |
| `analyzer.py` | Bag parsing, phase extraction, and report generation |
| `metrics.py` | Metric implementations |
| `outcomes.py` | Mission protocol and semantic outcome evaluation |
| `visualizer.py` | Static plots and 3D replay |
| `manifests.py` | External algorithm manifest loading and validation |

## Task and metric profiles

| Metric profile | Tasks | Primary focus |
|---|---|---|
| `hover_regulation` | `hover` | Position stability, attitude/thrust variation, oscillation |
| `trajectory_tracking` | `analytic_circle`, `analytic_figure8`, `analytic_spiral` | Time-aligned reference tracking and spatial shape |
| `trajectory_generation` | `discrete_circle`, `discrete_figure8` | Planner output quality, waypoint geometry, continuity, and execution |
| `online_mission` | `sequential_goal_mission`, `reference_path_mission` | Mission completion, timing, collision evidence, and tracking |

Every task reports its fixed metric set. Missing inputs produce unavailable metrics with provenance instead of silently switching to a different data source. The framework does not combine heterogeneous metrics into a global score.

## Topic contract

Common recorded topics begin with:

```text
/mavros/local_position/odom
/mavros/state
```

The selected controller, planner, task, external manifest, and `--extra-topics` extend that set. Duplicate topics are removed.

### Controller lifecycle

Benchmark-compatible controllers publish `std_msgs/Int8` on `/flight_state`:

| Code | State |
|---:|---|
| 0 | `WAITING_FOR_CONNECTED` |
| 1 | `WAITING_FOR_OFFBOARD` |
| 2 | `TAKEOFF` |
| 3 | `MISSION_EXECUTION` |
| 4 | `LANDING` |
| 5 | `LANDED` |
| 6 | `EMERGENCY` |

`/flight_state` is required for phase extraction. It is an input to the runner's lifecycle, not a replacement for the runner state itself.

### Unified planner output

Planners publish `opendrone/PlannerOutput` on `/planner/output`, either natively or through an adapter. For every point:

```text
absolute_desired_time = trajectory_start_time + point.time_from_start
```

Contract rules:

- `header.frame_id` identifies one local ROS ENU frame for all spatial fields in the message.
- Position is required; other fields are optional.
- `valid_mask` declares every valid derivative, yaw, yaw-rate, or angular-velocity field.
- Consumers must ignore fields whose bits are not set.
- `trajectory_id` changes when a new native trajectory becomes active and remains stable across windows of that trajectory.
- `is_horizon=true` means a newer message replaces the previous rolling window.
- A non-horizon multi-point message is a time sequence; a single point remains active until replaced.
- `trajectory_status` describes only the local trajectory lifecycle.

Native algorithm IDs, maps, solver diagnostics, and unsampled polynomial or B-spline data remain on algorithm-specific diagnostic topics.

### Mission protocol

Mission-capable planners publish `opendrone/MissionState` on `/planner/mission_state`. Mission type, completed item count, total item count, and success must remain internally consistent and monotonic. Only a success state matching the active task may end the evaluation.

The original mission path is `/waypoint_generator/waypoints` (`nav_msgs/Path`). Its frame must be compatible with MAVROS local odometry. The evaluator uses geometry as evidence; it does not advance goals or manufacture mission progress.

### Collision evidence

Collision events are read from `/flight_eval/contact_pulse`. Continuous contact with the same obstacle is merged into an episode using `collision_episode_gap`. Ground contact is excluded by the simulation plugin. Reaching `max_collision_episodes` ends the task safely.

## Runner lifecycle

```text
PREPARING -> RECORDING -> WAITING_FOR_CONTROLLER -> STARTING_PLANNER
  -> WAITING_FOR_PLANNER -> EXECUTING -> REQUESTING_LAND
  -> WAITING_FOR_LANDED -> CAPTURING_ARTIFACTS -> COMPLETED
```

An emergency transitions to `ABORTING` and then artifact capture. A timeout produces `FAILED`; ROS shutdown or user interruption produces `INTERRUPTED`.

The runner launches rosbag, controller, and planner in independent process groups. Cleanup escalates from `SIGINT` to `SIGTERM` and then `SIGKILL`. Remaining process groups after escalation make the run fail with `process_cleanup_incomplete` rather than reporting false completion.

In batch mode, the external Gazebo/MAVROS environment is not owned by an individual case. Each case owns its PX4 process and isolated root filesystem. Batch termination cleans the current case and, unless `--keep-gazebo` is set, attempts to close the external environment at the end.

## Phase extraction and analysis

The analyzer first locates the task phase from `/flight_state`, then applies the task's execution-start policy:

- `hover`: mission-state phase start.
- `analytic_*`: first valid reference or planner output.
- `discrete_*`: first trigger, waypoint, or planner output.
- mission tasks: first trigger, waypoint, or planner output.

If no required task signal arrives, `execution_started` is false. An emergency truncates the phase at its timestamp. Analysis and visualization reject bags without `/flight_state` because they cannot establish a comparable execution interval.

Reference samples are reconstructed from `/planner/output`. Rolling horizons are deduplicated with trajectory identity and absolute desired time. Reference gaps are preserved rather than interpolated across unrelated segments.

Time-aligned tracking and spatial path shape are intentionally separate measurements. Tracking metrics penalize lag and phase error. Spatial shape metrics compare continuous polylines without time pairing and therefore describe geometry rather than traversal timing or order.

## Result semantics

```yaml
run_status:
  status: completed
  reason: artifacts captured

task_outcome:
  status: succeeded
  reason: goal_reached
  completion_time: 23.6
  evidence: {}
```

`run_status` is one of `completed`, `failed`, or `interrupted`. `task_outcome` is one of `succeeded`, `not_succeeded`, `unknown`, or `not_applicable`. Timed tasks normally have no semantic endpoint and therefore use `not_applicable`.

## External algorithm manifests

External algorithms should be registered with YAML rather than by editing the built-in registries. The schema is [`algorithm_manifest.schema.yaml`](../schemas/algorithm_manifest.schema.yaml), and a complete example is [`external_algorithm_manifest.yaml`](../examples/external_algorithm_manifest.yaml).

```yaml
version: 1
algorithms:
  my_planner_depth:
    kind: planner
    launch:
      package: my_planner
      file: flight_eval.launch
    args:
      sensor: depth_camera
    tasks:
      - sequential_goal_mission
    record_topics:
      - /my_planner/diagnostics
```

Required algorithm fields are its name, `kind`, `launch.package`, and `launch.file`. Optional fields provide default launch arguments, a task allowlist, and diagnostic recording topics.

Load a manifest for one run:

```bash
python3 -m flight_eval run \
  --manifest /path/to/my_package/flight_eval.yaml \
  --controller my_controller \
  --planner my_planner_depth \
  --task sequential_goal_mission
```

Batch files accept a root-level `manifests` list. The list, analyze, and visualize commands also accept `--manifest`. Loaded manifests and resolved launch arguments are archived with the run.

## Adding built-in behavior

- Add a task only when it introduces distinct experiment semantics; implement it in `tasks.py` and register it in `TASK_REGISTRY`.
- Add metrics in `metrics.py`, then compose them in the owning task instead of creating switchable report views.
- Add a built-in launch entry to the appropriate controller or planner registry.
- Keep benchmark wrappers under `opendrone/launch/benchmarks/{controller_benchmark,trajectory_generation_benchmark,online_planner_benchmark}`.
- Keep algorithm-specific conversion in a launch-level adapter; do not expand the common message with solver-specific fields.
- Update the schema and example together when the manifest format changes.

Validate changes with dry-run configuration checks, recorded-bag reanalysis, and the smallest representative SITL task before running a full experiment matrix.
