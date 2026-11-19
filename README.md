[English](README.md) | [简体中文](README_CN.md)

# OpenDrone

[![ROS Noetic Catkin Build](https://github.com/Tfly6/OpenDrone/actions/workflows/catkin-build.yml/badge.svg?branch=main)](https://github.com/Tfly6/OpenDrone/actions/workflows/catkin-build.yml)

OpenDrone is a ROS 1 integration workspace for PX4 multicopter simulation. It brings controllers, trajectory generators, online planners, Gazebo Classic assets, and a reproducible flight-evaluation pipeline behind a common set of launch files and messages.

This repository contains the code used to run and analyze the paper experiments. See the [paper experiment code README](opendrone/data_code/README.md) for the experiment design, run instructions, and analysis workflow; the complete results are distributed separately in the paper's data package.

The repository is intended for simulation, algorithm integration, and research benchmarking. It is not a flight-certified stack; validate every controller and planner in simulation before using it on real hardware.

**Contents:** [Architecture](#architecture) · [Algorithms](#included-algorithms) · [Environment](#supported-environment) · [Installation](#installation) · [Quick start](#quick-start) · [Evaluation](#evaluation-and-benchmarking) · [Integration](#integrating-another-algorithm) · [Troubleshooting](#troubleshooting)

## Highlights

- Six controller backends: PID, LQR, linear MPC, nonlinear MPC, and two geometric controllers.
- Seven trajectory-generation and online-planning families across nine launch configurations.
- A unified planner output, `opendrone/PlannerOutput`, consumed by compatible controllers.
- Depth-camera and Livox Mid-360 simulation configurations.
- Repeatable single-run and batch evaluation with rosbag capture, metrics, plots, and 3D replay.
- Category-oriented benchmark launch wrappers that keep upstream algorithm launches intact.

## Architecture

```text
waypoint / mission / sensor data
               |
               v
     trajectory generator or planner
               |
       planner_adapter.py
               |
      /planner/output
      (PlannerOutput)
               |
               v
           controller
               |
     MAVROS attitude/thrust setpoints
               |
               v
          PX4 SITL + Gazebo
```

[`opendrone/scripts/planner_adapter.py`](opendrone/scripts/planner_adapter.py) normalizes native planner messages such as `PositionCommand`, `Bspline`, `PolynomialTrajectory`, and `PolyTraj`. Each output point uses `valid_mask` to declare which position, velocity, acceleration, jerk, snap, yaw, yaw-rate, and angular-velocity fields are valid. Local trajectory lifecycle and mission completion are deliberately separate concepts.

## Included algorithms

### Controllers

| Controller | Package | Launch file | Origin |
|---|---|---|---|
| Geometric control (Lee) | `se3_lee` | `sitl_se3_lee.launch` | [mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers) |
| Geometric control (Hopf) | `se3_hopf` | `sitl_se3_hopf.launch` | [se3_controller](https://github.com/HITSZ-MAS/se3_controller) |
| PID / cascade PID | `pid_controller` | `sitl_pid_controller.launch` | OpenDrone integration |
| LQR | `lqr_controller` | `sitl_lqr_controller.launch` | [lqr-tracking](https://github.com/llanesc/lqr-tracking) |
| Linear MPC | `mav_linear_mpc` | `sitl_mpc_controller.launch` | [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) |
| Nonlinear MPC | `mav_nonlinear_mpc` | `sitl_nmpc_controller.launch` | [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) |

### Trajectory generators and online planners

| Integration | Launch file | Sensor requirement | Origin |
|---|---|---|---|
| MAV trajectory generation | `sitl_mav_trajectory_planner.launch` | None | [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) |
| RPG polynomial trajectory | `sitl_rpg_trajectory.launch` | None | [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control) |
| Fast-Planner | `sitl_fast_planner.launch` | Depth camera | [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) |
| EGO-Planner | `sitl_ego_planner.launch` | Depth camera | [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) |
| EGO-Planner + Mid-360 | `sitl_ego_planner_mid360.launch` | 3D LiDAR | [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) |
| EGO-Planner v2 | `sitl_ego_planner_v2.launch` | Depth camera | [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) |
| Air-FAR | `sitl_airfar_planner.launch` | Depth camera | [Air-FAR](https://github.com/Bottle101/Air-FAR) |
| Air-FAR + Mid-360 | `sitl_airfar_planner_mid360.launch` | 3D LiDAR | [Air-FAR](https://github.com/Bottle101/Air-FAR) |
| SUPER | `sitl_super_planner_od.launch` | 3D LiDAR | [SUPER](https://github.com/hku-mars/SUPER) |

Polynomial generators do not provide obstacle avoidance. Sensor topics, frames, maps, and planner parameters must match the selected simulated vehicle.

## Repository layout

```text
OpenDrone/
|-- controller/                    controller packages
|-- planner/                       trajectory generators and online planners
|-- opendrone/
|   |-- launch/                    integration launch files
|   |   `-- benchmarks/            evaluation-oriented launch wrappers
|   |-- msg/                       unified planner messages
|   |-- rviz/                      RViz configurations
|   |-- scripts/
|   |   `-- flight_eval/           experiment and analysis framework
|   |-- sitl_config/               PX4/Gazebo assets (Git submodule)
|   `-- src/                       base nodes and basic examples
|-- utils/                         shared messages, libraries, and plugins
|-- shell/                         setup, recording, and landing helpers
|-- cmake/Modules/                 CMake dependency finders
`-- .github/workflows/             ROS Noetic build verification
```

## Supported environment

The continuously tested target is:

| Component | Tested target |
|---|---|
| Operating system | Ubuntu 20.04 |
| ROS | ROS Noetic |
| Simulator | PX4 SITL with Gazebo Classic |
| Build system | catkin tools |

Some upstream packages may also build on ROS Melodic, but the current repository CI does not verify that configuration.

## Installation

### 1. Prepare PX4 and ROS

Install ROS Noetic, MAVROS, and a PX4-Autopilot version that provides Gazebo Classic SITL. Confirm that the standard PX4 launch works before integrating OpenDrone:

```bash
roslaunch px4 mavros_posix_sitl.launch
```

Follow the [PX4 ROS 1 / MAVROS documentation](https://docs.px4.io/main/en/ros/mavros_installation) for the base environment.

### 2. Create a catkin workspace and clone recursively

```bash
sudo apt update
sudo apt install -y python3-catkin-tools python3-rosdep git

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/Tfly6/OpenDrone.git
```

If the repository was cloned without `--recursive`, initialize its required submodules:

```bash
cd ~/catkin_ws/src/OpenDrone
git submodule update --init --recursive
```

The required submodules are `utils/catkin_simple`, `utils/mav_comm`, and `opendrone/sitl_config`.

### 3. Install native and ROS dependencies

```bash
sudo apt install -y \
  libgoogle-glog-dev libgflags-dev libeigen3-dev \
  libarmadillo-dev liblapacke-dev liblapack-dev libopenblas-dev \
  libdw-dev libopencv-dev libyaml-cpp-dev

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
cd ~/catkin_ws
rosdep install --from-paths src/OpenDrone --ignore-src --rosdistro noetic -r -y
```

Install NLopt 2.7.1, which is used by the polynomial trajectory packages:

```bash
cd /tmp
git clone --branch v2.7.1 --depth 1 https://github.com/stevengj/nlopt.git
cmake -S nlopt -B nlopt/build
cmake --build nlopt/build -j"$(nproc)"
sudo cmake --install nlopt/build
```

### 4. Build

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin init
catkin build
source devel/setup.bash
```

MPC packages can significantly increase build time. Skip them when they are not needed:

```bash
catkin config --skiplist mav_disturbance_observer mav_linear_mpc mav_nonlinear_mpc
catkin build
```

### 5. Deploy the simulation assets

After PX4 SITL has been built, copy OpenDrone's models, worlds, launch files, and available plugins into the detected PX4 Gazebo Classic layout:

```bash
cd ~/catkin_ws/src/OpenDrone
./shell/gazeboSetup.bash /absolute/path/to/PX4-Autopilot
```

The script supports both `Tools/sitl_gazebo` and `Tools/simulation/gazebo-classic/sitl_gazebo-classic`. It copies files into the PX4 tree, so keep local PX4 customizations under version control or back them up first.

## Quick start

Every terminal below must source both ROS and the workspace.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Basic PX4 offboard example

Terminal 1:

```bash
roslaunch px4 mavros_posix_sitl.launch
```

Terminal 2:

```bash
rosrun opendrone offb_node
```

The example arms in offboard mode, climbs to approximately 2 m, and holds position.

### Geometric controller with a polynomial trajectory

Start PX4 SITL in terminal 1, then run:

```bash
roslaunch opendrone sitl_se3_hopf.launch
```

In another terminal:

```bash
roslaunch opendrone sitl_mav_trajectory_planner.launch
```

### EGO-Planner with a depth camera

Start a depth-camera PX4 vehicle/world in terminal 1:

```bash
roslaunch px4 outdoor_depth_camera.launch
```

Then start the controller and planner in separate terminals:

```bash
roslaunch opendrone sitl_se3_hopf.launch
```

```bash
roslaunch opendrone sitl_ego_planner.launch
```

For Mid-360 examples, first install the simulation model described by [Mid360_px4_sim_plugin](https://github.com/Tfly6/Mid360_px4_sim_plugin), then use `outdoor_mid360.launch` with a `*_mid360.launch` planner entry.

To request landing:

```bash
cd ~/catkin_ws/src/OpenDrone
./shell/trigger_land.sh
```

## Evaluation and benchmarking

OpenDrone includes `flight_eval`, which can run controller, trajectory-generation, and online-planning tasks; record bags; recompute metrics; create plots; and execute YAML experiment matrices.

```bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH

python3 -m flight_eval list-controllers
python3 -m flight_eval list-planners
python3 -m flight_eval list-tasks
```

Start with the [flight evaluation README](opendrone/scripts/flight_eval/README.md). Its user and developer guides cover environment preparation, experiment operation, task semantics, metrics, and external algorithm manifests. Benchmark wrappers are organized directly under `opendrone/launch/benchmarks/` by category.

## Integrating another algorithm

For a planner, publish `opendrone/PlannerOutput` on `/planner/output`, or add a conversion path to [`planner_adapter.py`](opendrone/scripts/planner_adapter.py). Observe these interface rules:

- Set `header.frame_id` to the local ENU frame used by MAVROS odometry.
- Set `valid_mask` for every populated derivative or yaw field.
- Keep `trajectory_id` stable across windows from the same active trajectory.
- Use `trajectory_status` only for local trajectory lifecycle, not waypoint or mission completion.
- Publish mission progress separately on `/planner/mission_state` when the task requires it.

For benchmark integration, add a wrapper under `opendrone/launch/benchmarks/` and an algorithm manifest entry. The detailed contract is in the [flight evaluation developer guide](opendrone/scripts/flight_eval/docs/developer-guide.md).

## Troubleshooting

- **A package is missing during configuration:** verify `git submodule status`; each required entry should have a checked-out commit rather than a leading `-`.
- **PX4 models or worlds are missing:** build PX4 SITL, then rerun `shell/gazeboSetup.bash` with the correct PX4 root.
- **MAVROS does not connect:** confirm the FCU URL and UDP/TCP ports match the PX4 and Gazebo configuration.
- **A planner receives no sensor data:** check topic names and TF frames in its launch arguments and RViz configuration.
- **The full build is too slow:** use the catkin skiplist shown above and enable only the packages needed for the experiment.

## Upstream projects and license

OpenDrone integrates and adapts work from the projects linked in the algorithm tables. Their source files and dependencies may carry their own licenses and citation requirements; consult the corresponding directories and upstream repositories before redistribution or publication.

OpenDrone's own top-level code is provided under the [MIT License](LICENSE).
