[EN](./README.md)|[简体中文](./README_CN.md)

# OpenDrone

ROS1-based PX4 UAV simulation. This project references multiple open-source projects and integrates them into PX4 SITL, including:

## Project Tree

```text
OpenDrone/
├── cmake/                # CMake modules and dependency finder scripts for third-party libs such as glog, gflags, and OpenBLAS
│   └── Modules/
├── controller/           # Controller collection, packaged as ROS packages for easy switching in PX4 SITL
├── opendrone/            # Main ROS package
│   ├── config/           # Parameter configuration
│   ├── include/          # Public header files
│   ├── launch/           # One-click launch files for SITL, controllers, planners, and tests
│   ├── rviz/             # RViz configurations
│   ├── scripts/          # Python helper scripts, e.g., message conversion, camera pose publishing, point cloud processing
│   ├── sitl_config/      # Models, worlds, plugin lists, and simulation settings for PX4/Gazebo SITL
│   └── src/              # Basic examples and helper tool implementations
├── planner/              # Planner collection
├── shell/                # Common scripts
├── utils/                # Shared dependencies, message definitions, math tools, visualization, and utility libraries reused by controllers and planners
├── CMakeLists_Template.txt
├── LICENSE
└── README.md
```

> Note: Most ROS packages in this repository follow a similar structure, e.g., **cfg/** for dynamic parameter config, **include/** for headers, **launch/** for launch files, **src/** for source code, and **test/** for tests or demos.

**Controllers (controller folder)**

- **geometric_controller**:

  - **se3_lee**: Referenced from [Jaeyoung-Lim/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers). See [README.md](./controller/geometric_controller/se3_lee/README.md).

  Launch:

  ```bash
  roslaunch opendrone sitl_se3_lee.launch # default Nonlinear GeometricControl
  # land
  # ./shell/trigger_land.sh
  ```

  - **se3_hopf**: Referenced from [HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller). See [README.md](./controller/geometric_controller/se3_hopf/README.md). (**Recommended**)

  Launch:

  ```bash
  roslaunch opendrone sitl_se3_hopf.launch
  # land
  # ./shell/trigger_land.sh
  ```

- **pid_controller**: Includes simple PID (for learning only) and cascade PID. See [README.md](./controller/pid_controller/README.md).

  Launch:

  ```bash
  roslaunch opendrone sitl_pid_controller.launch # default cascade PID
  # land
  # ./shell/trigger_land.sh
  ```

- **lqr_controller**: Referenced from [llanesc/lqr-tracking](https://github.com/llanesc/lqr-tracking). A simple LQR controller. See [README.md](./controller/lqr_controller/README.md).

  Launch:

  ```bash
  roslaunch opendrone sitl_lqr_controller.launch
  ```

- **mpc_controller**: Referenced from [ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw), including linear MPC and nonlinear MPC. This controller is still experimental.

  Launch:

  ```bash
  # linear MPC
  roslaunch opendrone sitl_mpc_controller.launch
  # nonlinear MPC
  roslaunch opendrone sitl_nmpc_controller.launch
  ```

**Planners (planner folder)**

- **polynomial trajectory generation**: Referenced from [ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation). This is an optimization-based polynomial trajectory generator and does not support real-time planning and obstacle avoidance.

  Launch:

  ```bash
  roslaunch opendrone sitl_mav_trajectory_planner.launch
  ```

- **fast_planner**: Referenced from [HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner). Requires a UAV with a depth camera.

  Launch:

  ```bash
  roslaunch opendrone sitl_fast_planner.launch
  ```

- **ego_planner**: Referenced from [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm). Requires a UAV with a depth camera or 3D LiDAR. (**Recommended**)

  Launch:

  ```bash
  roslaunch opendrone sitl_ego_planner.launch # depth camera
  roslaunch opendrone sitl_ego_planner_mid360.launch # LiDAR
  ```

- **ego_plannerV2**: Referenced from [ZJU-FAST-Lab/EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2). Requires a UAV with a depth camera.

  Launch:

  ```bash
  roslaunch opendrone sitl_ego_planner_v2.launch # depth camera
  ```

- **airfar_planner**: Referenced from [Bottle101/Air-FAR](https://github.com/Bottle101/Air-FAR). Requires a UAV with a depth camera or 3D LiDAR. This planner is still experimental.

  Launch:

  ```bash
  roslaunch opendrone sitl_airfar_planner.launch # depth camera
  roslaunch opendrone sitl_airfar_planner_mid360.launch # LiDAR
  ```

> Supported on Ubuntu 18.04 + ROS Melodic, and Ubuntu 20.04 + ROS Noetic.

## 1. Preparation

- **Before use, you must set up** the [PX4 UAV simulation environment](https://docs.px4.io/main/zh/sim_gazebo_classic/)

- **Create a workspace**
If you have not created a workspace yet, run the following commands. Skip this step if your workspace is already set up.

```bash
sudo apt-get install python-catkin-tools python-rosinstall-generator -y

# For ROS Noetic:
# sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws && catkin init # initialize workspace
catkin build
```

- **Install dependencies**

```bash
sudo apt install libgoogle-glog-dev libgflags-dev libeigen3-dev libarmadillo-dev liblapacke-dev
sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-laser-geometry ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-rviz
```

**Install the NLopt library**

```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

## 2. Build

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/Tfly6/OpenDrone.git
cd ~/catkin_ws
# Building MPC-related packages may take more than 10 minutes.
# If you do not need MPC, you can add them to the build skiplist (same idea for other unused packages).
# catkin config --skiplist mav_control_interface mav_disturbance_observer mav_linear_mpc mav_nonlinear_mpc
catkin build
```

## 3. Run

- **Configure simulation**

```bash
cd ~/catkin_ws/src/OpenDrone/shell
./gazeboSetup.bash ${YOUR_PX4_PATH}
```

### Example 1: Official basic example

(The [opendrone/src/basic_example](opendrone/src/basic_example) directory contains basic examples only, without controllers or planners.)

- Terminal 1: start Gazebo simulation

```bash
roslaunch px4 mavros_posix_sitl.launch
```

- Terminal 2: start the official example, take off to 2 meters, then hover

```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun opendrone offb_node
```

### Example 2: geometric_controller + polynomial trajectory generation

Fly a circle.

- Terminal 1: start Gazebo simulation

```bash
roslaunch px4 mavros_posix_sitl.launch
```

- Terminal 2: start geometric_controller

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_se3_hopf.launch
```

- Terminal 3: start polynomial trajectory generation

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_mav_trajectory_planner.launch
```

Demo video:

[bilibili](https://www.bilibili.com/video/BV1EZTszKEtJ/?share_source=copy_web&vd_source=649164de6e400405dc9e781456725af7)

### Example 3: geometric_controller + ego_planner + depth camera

Real-time planning and obstacle avoidance.

- Terminal 1: start Gazebo simulation

```bash
roslaunch px4 outdoor_depth_camera.launch # or use your own world/launch
```

- Terminal 2: start geometric_controller

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_se3_hopf.launch
```

- Terminal 3: start ego-planner

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_ego_planner.launch
```

Demo video:

[bilibili](https://www.bilibili.com/video/BV17ZTszKEea/?share_source=copy_web&vd_source=649164de6e400405dc9e781456725af7)

### Example 4: geometric_controller + ego_planner + Mid360

Real-time planning and obstacle avoidance.

- Configure Mid360 simulation according to this repository:

[Tfly6/Mid360_px4_sim_plugin: Plugin for the simulation of the Livox Mid-360 in Gazebo](https://github.com/Tfly6/Mid360_px4_sim_plugin)

- Terminal 1: start Gazebo simulation

```bash
roslaunch px4 outdoor_mid360.launch # or use your own world/launch
```

- Terminal 2: start geometric_controller

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_se3_hopf.launch
```

- Terminal 3: start ego-planner

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_ego_planner_mid360.launch
```

Demo video:

[Mid360 + ego-planner PX4 UAV Gazebo simulation demo (bilibili)](https://www.bilibili.com/video/BV1dHFsz5EEh/?spm_id_from=333.1387.homepage.video_card.click&vd_source=d59e7d5891b69289e548bcfb7a4948a0)

## Related Papers

[1] Lee, Taeyoung, Melvin Leoky, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.

[2] Faessler, Matthias, Antonio Franchi, and Davide Scaramuzza. "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories." IEEE Robot. Autom. Lett 3.2 (2018): 620-626.

[3] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," in Proc. of the IEEE Intl. Conf. on Robot. and Autom. (ICRA), Shanghai, China, May 2011, pp. 2520-2525.

[4] X. Zhou, Z. Wang, H. Ye, C. Xu, and F. Gao, "EGO-Planner: An ESDFfree gradient-based local planner for quadrotors," IEEE Robotics and Automation Letters, vol. 6, no. 2, pp. 478-485, 2021.

[5] Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System. Mina Kamel, Thomas Stastny, Kostas Alexis and Roland Siegwart. Robot Operating System (ROS) The Complete Reference Volume 2. Springer 2017.

[6] Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles. Mina Kamel, Michael Burri and Roland Siegwart. arXiv:1611.09240.

[7] B. He, G. Chen, C. Fermuller, Y. Aloimonos and J. Zhang, "Air-FAR: Fast and Adaptable Routing for Aerial Navigation in Large-Scale Complex Unknown Environments," 2025 IEEE International Conference on Robotics and Automation (ICRA).

[8] Xin Zhou et al., Swarm of micro flying robots in the wild. *Sci. Robot.* 7, eabm5954 (2022).

[9] Foehn, Philipp and Scaramuzza, Davide. (2018). Onboard State Dependent LQR for Agile Quadrotors. 10.1109/ICRA.2018.8460885.

[10] B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight," in *IEEE Robotics and Automation Letters*, vol. 4, no. 4, pp. 3529-3536, Oct. 2019.

