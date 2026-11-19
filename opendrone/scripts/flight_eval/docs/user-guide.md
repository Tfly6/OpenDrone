[English](user-guide.md) | [简体中文](user-guide_CN.md)

# Flight Evaluation User Guide

This guide covers experiment selection and operation. For interfaces and implementation rules, see the [developer guide](developer-guide.md).

## Before a run

1. Build OpenDrone and source the catkin workspace.
2. Add `opendrone/scripts` to `PYTHONPATH`.
3. Start a simulation environment whose world, vehicle sensors, transport ports, and MAVROS URL match the experiment.
4. Confirm that MAVROS is connected and ROS time advances.
5. Use the list commands to verify the names supported by the current checkout.

```bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH
cd src/OpenDrone/opendrone/scripts

python3 -m flight_eval list-controllers
python3 -m flight_eval list-planners
python3 -m flight_eval list-tasks
```

## Choose a task

| Task | Primary purpose | Planner |
|---|---|---|
| `hover` | Hover regulation | `none` |
| `analytic_circle` | Continuous circle-reference tracking | Automatically uses `analytic_reference` |
| `analytic_figure8` | Continuous figure-eight tracking | Automatically uses `analytic_reference` |
| `analytic_spiral` | Continuous spiral tracking | Automatically uses `analytic_reference` |
| `discrete_circle` | Trajectory generation from circle waypoints | Required |
| `discrete_figure8` | Trajectory generation from figure-eight waypoints | Required |
| `sequential_goal_mission` | Execute an ordered sequence of goals | Required |
| `reference_path_mission` | Execute a continuous reference path | Required |

For timed tasks, `--duration` is the evaluation window. For finite trajectory and mission tasks, it is the completion deadline.

## Single runs

The general form is:

```bash
python3 -m flight_eval run \
  --controller CONTROLLER \
  --task TASK \
  --planner PLANNER
```

Common options:

| Option | Purpose |
|---|---|
| `--takeoff-height METERS` | Override the task's default takeoff height |
| `--duration SECONDS` | Set the evaluation window or deadline |
| `--record-dir PATH` | Select the output directory |
| `--run-name NAME` | Set the run subdirectory name |
| `--controller-arg key:=value` | Pass a launch argument to the controller |
| `--planner-arg key:=value` | Pass a launch argument to the planner |
| `--extra-topics TOPIC ...` | Add rosbag topics |
| `--no-land` | Do not request automatic landing |
| `--keep-gazebo` | Keep the external Gazebo environment after completion |

Examples:

```bash
python3 -m flight_eval run \
  --controller pid_controller \
  --task hover \
  --takeoff-height 1.5 \
  --duration 15

python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_figure8 \
  --planner rpg_trajectory

python3 -m flight_eval run \
  --controller mav_nonlinear_mpc \
  --task sequential_goal_mission \
  --planner super_planner_od \
  --max-collision-episodes 3
```

Run `python3 -m flight_eval run --help` for the authoritative option list.

## Batch experiments

Batch mode expands controller, planner, and repetition lists into a case matrix. Gazebo and MAVROS are external to the batch and remain available across cases; the batch starts a separate PX4 SITL process and root filesystem for each case.

### Prepare a UDP environment

The YAML file must identify the PX4 source tree and use a Gazebo/MAVROS/transport configuration that remains constant for the batch. Generate the environment files first:

```bash
python3 -m flight_eval prepare-environment \
  --config ./flight_eval/examples/batch_example.yaml
```

The command prints the generated paths. In the terminal that will own the external environment, source `environment.env` and run `launch_command.sh`:

```bash
source /tmp/flight_eval_udp_environment/environment.env
bash /tmp/flight_eval_udp_environment/launch_command.sh
```

This launch contains Gazebo, the vehicle model, and MAVROS, but not PX4. The generated Iris override uses the configured UDP port without modifying the PX4 source or build directory.

### Validate and run the matrix

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml \
  --dry-run

python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml
```

Dry-run validates names, task compatibility, configuration, and case expansion without starting ROS processes. By default, a failed case stops the batch; use `--continue-on-error` when later cases should still run. Use `--keep-gazebo` when the external environment should remain active after the batch.

### Minimal matrix shape

```yaml
version: 1
name: controller_comparison
defaults:
  simulation:
    gazebo:
      world: empty
      sensor: none
      respawn_gazebo: true
    mavros:
      fcu_url: udp://:14540@localhost:14557
      respawn_mavros: true
    px4:
      source_dir: /absolute/path/to/PX4-Autopilot
    transport:
      gazebo_px4: udp
      port: 14560
      environment_dir: /tmp/flight_eval_udp_environment
  run:
    takeoff_height: 2.0
    duration: 20
    auto_land: true
experiments:
  - name: hover
    task: hover
    controllers: [se3_hopf, pid_controller]
    planners: [none]
    repeat: 3
```

`controllers × planners × repeat` determines the number of cases. A single entry may be an object with `name` and `args`. See [`batch_example.yaml`](../examples/batch_example.yaml) for all supported simulation and run fields.

## Analyze recorded data

Single bag:

```bash
python3 -m flight_eval analyze \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory
```

If `run_metadata.json` is next to the bag, analysis restores the original task duration, takeoff height, runtime outcome, and algorithm context. Otherwise, supply the missing values on the command line.

Reanalyze every case in a batch and recompute the outcome using the current evaluator:

```bash
python3 -m flight_eval analyze \
  --batch-dir eval_runs/batches/my_batch \
  --recompute-outcome
```

Recomputation overwrites each case's report, summary, and core plots. The original bag remains unchanged.

## Visualize recorded data

```bash
python3 -m flight_eval visualize \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task sequential_goal_mission \
  --planner fast_planner_kino
```

Add `--scope full` to render takeoff through landing instead of only the task execution phase. Add `--gif` for `plots/flight_replay.gif`; `--gif-fps` and `--gif-duration` control its timing.

Core artifacts may include:

```text
plots/trajectory_3d.png
plots/trajectory_xy.png
plots/position_tracking.png
plots/position_tracking_error.png
plots/attitude_thrust.png
plots/yaw_tracking_error.png
plots/metrics_overview.png
plots/flight_replay.gif
```

Plots that require unavailable reference fields are omitted. Hover tasks do not create path-shape plots.

## Result directories

Single runs default to:

```text
eval_runs/{controller}_{task}_{planner}_{timestamp}/
```

Batch runs default to:

```text
eval_runs/batches/{batch_name}_{timestamp}/
|-- batch_plan.json
|-- batch_summary.json
|-- cases/
|   `-- {case}/
|       `-- px4_rootfs/
`-- logs/
```

Each completed case normally stores its bag, `run_metadata.json`, `report.json`, `agent_summary.md`, parameter snapshots, and plots. External algorithm manifests are archived with the case.

## Troubleshooting

- **No controller or planner output:** verify the selected algorithm supports the task and inspect the launch output.
- **MAVROS never becomes ready:** check the FCU URL, transport mode, and port agreement among PX4, Gazebo, and MAVROS.
- **ROS time stalls:** confirm Gazebo is unpaused and `/clock` is advancing.
- **No analysis phase:** the bag must contain `/flight_state` and the state machine must reach the task phase.
- **No reference metrics:** confirm `/planner/output` was recorded and its points set the required `valid_mask` bits.
- **A batch cannot change world or sensor:** split configurations that need different external Gazebo environments into separate batch files.
