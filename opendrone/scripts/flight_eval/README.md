[English](README.md) | [简体中文](README_CN.md)

# OpenDrone Flight Evaluation

`flight_eval` runs and analyzes reproducible PX4 SITL experiments for OpenDrone controllers, trajectory generators, and online planners. It records the ROS evidence needed for each task and produces structured results without assigning an overall score or algorithm ranking.

## What it provides

- Controller, trajectory-generation, and online-mission tasks.
- Single runs and YAML experiment matrices.
- Automatic rosbag capture and run metadata.
- Offline reanalysis without rerunning the simulation.
- Trajectory, tracking, attitude, thrust, and metric plots.
- Optional 3D flight replay as a GIF.
- YAML manifests for algorithms maintained outside this repository.

## Requirements

Build OpenDrone and source the ROS workspace first. The Python analysis dependencies are listed in `requirements.txt`:

```bash
python3 -m pip install -r requirements.txt
```

Add this directory's parent to `PYTHONPATH`:

```bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH
cd src/OpenDrone/opendrone/scripts
```

## Discover available components

The registries in the current checkout are the source of truth:

```bash
python3 -m flight_eval list-controllers
python3 -m flight_eval list-planners
python3 -m flight_eval list-tasks
```

Use the built-in help for the complete and current option list:

```bash
python3 -m flight_eval --help
python3 -m flight_eval run --help
python3 -m flight_eval batch --help
python3 -m flight_eval analyze --help
python3 -m flight_eval visualize --help
```

## Quick start

Start a compatible Gazebo, PX4 SITL, and MAVROS environment before running an experiment.

Controller hover:

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task hover \
  --duration 20
```

Analytic trajectory tracking:

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task analytic_circle
```

Trajectory generation from discrete waypoints:

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory
```

Online mission:

```bash
python3 -m flight_eval run \
  --controller mav_linear_mpc \
  --task sequential_goal_mission \
  --planner fast_planner_kino
```

## Analyze and visualize a recording

Analyze an existing bag:

```bash
python3 -m flight_eval analyze \
  --bag /path/to/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory
```

Generate plots and an optional flight replay:

```bash
python3 -m flight_eval visualize \
  --bag /path/to/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory \
  --scope full --gif
```

## Batch experiments

Validate the matrix without starting ROS processes:

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml \
  --dry-run
```

Run it after preparing the matching external simulation environment:

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml
```

The example configuration is intentionally comprehensive. Copy it and keep only the experiments and simulation settings needed for a real batch.

## Output

A single run is stored under `eval_runs/` by default. Its artifacts normally include:

```text
flight_test*.bag
run_metadata.json
report.json
agent_summary.md
rosparams.json
parameter_snapshot.json
plots/
```

Batch runs additionally contain the expanded plan, a summary, per-case directories, PX4 logs, and isolated PX4 root filesystems.

`run_status` describes whether the execution and artifact-capture workflow completed. `task_outcome` independently describes whether a finite task reached its semantic goal. Neither field is an overall algorithm rating.

## Documentation

- [User guide](docs/user-guide.md): task selection, single runs, batch environments, analysis, visualization, and outputs.
- [Developer guide](docs/developer-guide.md): architecture, topic contracts, task semantics, metrics, lifecycle, and external algorithm manifests.
- [Batch example](examples/batch_example.yaml): complete experiment-matrix configuration.
- [External algorithm example](examples/external_algorithm_manifest.yaml): controller or planner registration outside the built-in registries.
- [Manifest schema](schemas/algorithm_manifest.schema.yaml): machine-readable manifest contract.

## Design principles

- A task defines the experiment semantics and its fixed metric set.
- `/planner/output` is the common trajectory interface.
- Reports expose measurements and provenance; they do not calculate a global score or ranking.
- Raw bags and metadata remain available so changed analysis logic can be applied offline.
- Runtime status and mission success remain separate results.

## Safety

This framework starts and stops ROS processes and may command arming, offboard mode, takeoff, and landing. Use it in an isolated SITL environment unless the launch files and safety behavior have been independently reviewed for the target hardware.
