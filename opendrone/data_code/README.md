# OpenDrone Paper Experiment Code

This directory contains code for running, analyzing, and plotting the paper experiments. The complete experimental results are distributed separately in the paper's data package; only a small synthetic example is kept here to quickly verify the analysis workflow.

## Experimental Design

| Group | Design and evaluation purpose | Trials |
| --- | --- | ---: |
| `hover` | Seven controllers track a fixed target to evaluate steady-state bias and position variation | 21 |
| `tracking` | Seven controllers track analytic circle, figure-eight, and spiral references to evaluate dynamic tracking error | 105 |
| `generation` | MAV/RPG generate references for circle and figure-eight tasks, which controllers then execute, to evaluate trajectory properties and downstream tracking | 140 |
| `online` | Run planning tasks in three environments with two sensor configurations to evaluate native status, geometric completion, and contact | 300 |
| `controlled` | Compare fixed single- and two-goal tasks in cylinder and forest environments to assess how task matching and status sources affect evaluation | 80 |
| `all` | All of the above trials | **646** |

The run manifest contains 646 flight attempts. Two analytic-reference tracking attempts and three trajectory-generation-and-execution attempts were excluded because they had too few valid samples, leaving 641 attempts in the paper. The rerun manifest retains all attempts; 18 synthetic rule-based cases and adapter checks are run separately.

`online` compares complete scenario packages: task geometry in the cylinder and forest scenarios varies with the scenario configuration. `controlled` uses the same task definitions, depth sensor, and time budget in both environments. It includes Hopf/PID and EGO/Fast-kino, with five repetitions for each combination.

`experiments/full_suite.json` specifies tasks, controllers, planners, repetition numbers, launch arguments, and waypoint parameters. The runner loads task parameters for each trial and executes the algorithm implementations in the current checkout. Parameter snapshots and implementation identifiers in the data package are used to verify the measurement conditions for archived results.

## Code Locations

| File/directory | Purpose |
| --- | --- |
| `run_suite.py` / `run_suite.sh` | Entry points for the full experiment manifest, planning, and execution |
| `experiments/full_suite.json` | Algorithm combinations, repetition counts, and run configuration for each experiment group |
| `experiments/task_*.yaml` | Single- and two-goal task definitions for controlled comparisons |
| `run_revision.py` / `run_revision.sh` | Entry points for evaluation-protocol checks, archive audits, and controlled-task execution |
| `revision_support/` | Reference reconstruction, task and contact audits, status-source review, and runtime support |
| `record_ground_truth.py` | Ground-truth sampling in Iris simulation |
| `summarize_planners.py`, `verify_release.py` | Read the external results package and reproduce paper statistics and the complete worked example |
| `build_evidence.py`, `draw_framework.py` | Generate tables and figures |
| `export_worked_example.py` | Export a complete example from an external raw bag |
| `samples/`, `verify_example.py` | Synthetic examples in two ten-row CSV files for verifying basic tracking-error calculations |
| `checks/` | Task, adapter, runner, and portability checks |

General execution and evaluation code is in `opendrone/scripts/flight_eval/`; controllers and planners are in the project's `controller/` and `planner/` directories, respectively.

## List Experiments and Verify the Small Example (Standard Library Only)

Run from this directory:

```bash
python3 run_suite.py list
python3 run_suite.py list online --sensor Mid360
python3 verify_example.py
python3 -m unittest checks.test_portability
```

## Plan and Run All Experiments

Requires ROS Noetic, catkin build artifacts for the current checkout, PX4 SITL, and Gazebo Classic. The entry point locates code relative to the current repository and searches upward for the workspace; set paths explicitly if needed:

```bash
export OPENDRONE_WORKSPACE=/path/to/catkin_ws
export PX4_SOURCE_DIR=/path/to/PX4-Autopilot

# Resolve all configuration and launch arguments without starting the simulation.
bash run_suite.sh plan all

# Select experiment groups as needed; "all" runs every experiment.
bash run_suite.sh run hover
bash run_suite.sh run tracking
bash run_suite.sh run generation
bash run_suite.sh run online
bash run_suite.sh run controlled
# bash run_suite.sh run all
```

`--sensor` and `--world` can narrow the scope; `--limit 1` runs one trial from each selected batch first. `--px4-source` overrides the PX4 path.

The entry point manages Gazebo/MAVROS, ground-truth sampling, and PX4 for each trial. Shut down any existing Gazebo/MAVROS processes before starting. Experiment records are written to `eval_runs/batches/` in the workspace. Plans, logs, and derived results are written to `paper_artifacts/` by default; override this location with `OPENDRONE_ARTIFACTS`.

Run only code checks and rule-based cases:

```bash
bash run_revision.sh check
```

## Analysis and Reproduction

Re-auditing bags or exporting an example requires the original archives and a ROS environment:

```bash
bash run_revision.sh audit --bags
bash run_revision.sh review
# After sourcing ROS/catkin:
python3 export_worked_example.py
```

Reproduce statistics and figures from the complete paper results package:

```bash
export OPENDRONE_DATA_DIR=/path/to/extracted_data_package/data
python3 summarize_planners.py
python3 verify_release.py
python3 build_evidence.py
```

CSV-based reproduction requires only the Python standard library. See `requirements.txt` for dependencies needed by figure generation and ROS entry points. Figures are written to `outputs/` by default; use `--output` to specify another directory. See [DATA_README.md](DATA_README.md) for descriptions of additional result files.
