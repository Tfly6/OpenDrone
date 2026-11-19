[English](README.md) | [简体中文](README_CN.md)

# OpenDrone

[![ROS Noetic Catkin Build](https://github.com/Tfly6/OpenDrone/actions/workflows/catkin-build.yml/badge.svg?branch=main)](https://github.com/Tfly6/OpenDrone/actions/workflows/catkin-build.yml)

OpenDrone 是一个面向 PX4 多旋翼仿真的 ROS 1 集成工作空间。项目将控制器、轨迹生成器、在线规划器、Gazebo Classic 仿真资源和可复现的飞行评测流程统一到一套 launch 文件与消息接口中。

本仓库主要用于仿真、算法集成和科研评测，并非经过飞行认证的系统。将任何控制器或规划器用于真机前，请先完成充分的仿真验证。

## 项目特点

- 集成 6 类控制器：PID、LQR、线性 MPC、非线性 MPC，以及两种几何控制器。
- 集成 7 个轨迹生成与在线规划项目。
- 使用统一的 `opendrone/PlannerOutput` 连接规划器和兼容控制器。
- 提供深度相机和 Livox Mid-360 仿真配置。
- 支持可复现的单次与批量评测，包括 rosbag、指标、图表和三维回放。
- 提供按评测类别组织的 launch 包装层，同时保留上游算法的原始启动方式。

## 系统架构

```text
航点 / 任务 / 传感器数据
            |
            v
      轨迹生成器或规划器
            |
    planner_adapter.py
            |
     /planner/output
     (PlannerOutput)
            |
            v
          控制器
            |
   MAVROS 姿态/推力设定值
            |
            v
       PX4 SITL + Gazebo
```

`planner_adapter.py` 将 `PositionCommand`、`Bspline`、`PolynomialTrajectory`、`PolyTraj` 等原生规划消息转换为统一输出。每个轨迹点通过 `valid_mask` 声明位置、速度、加速度、jerk、snap、偏航、偏航角速度和角速度等字段是否有效。局部轨迹生命周期与整项任务完成状态在接口中保持分离。

## 已集成算法

### 控制器

| 控制器 | ROS 包 | 启动文件 | 来源 |
|---|---|---|---|
| Lee 几何控制 | `se3_lee` | `sitl_se3_lee.launch` | [mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers) |
| Hopf 几何控制 | `se3_hopf` | `sitl_se3_hopf.launch` | [se3_controller](https://github.com/HITSZ-MAS/se3_controller) |
| PID / 串级 PID | `pid_controller` | `sitl_pid_controller.launch` | OpenDrone 集成 |
| LQR | `lqr_controller` | `sitl_lqr_controller.launch` | [lqr-tracking](https://github.com/llanesc/lqr-tracking) |
| 线性 MPC | `mav_linear_mpc` | `sitl_mpc_controller.launch` | [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) |
| 非线性 MPC | `mav_nonlinear_mpc` | `sitl_nmpc_controller.launch` | [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) |

### 轨迹生成器与在线规划器

| 集成项目 | 启动文件 | 传感器要求 | 来源 |
|---|---|---|---|
| MAV trajectory generation | `sitl_mav_trajectory_planner.launch` | 无 | [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) |
| RPG polynomial trajectory | `sitl_rpg_trajectory.launch` | 无 | [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control) |
| Fast-Planner | `sitl_fast_planner.launch` | 深度相机 | [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) |
| EGO-Planner | `sitl_ego_planner.launch` | 深度相机 | [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) |
| EGO-Planner + Mid-360 | `sitl_ego_planner_mid360.launch` | 三维激光雷达 | [ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) |
| EGO-Planner v2 | `sitl_ego_planner_v2.launch` | 深度相机 | [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) |
| Air-FAR | `sitl_airfar_planner.launch` | 深度相机 | [Air-FAR](https://github.com/Bottle101/Air-FAR) |
| Air-FAR + Mid-360 | `sitl_airfar_planner_mid360.launch` | 三维激光雷达 | [Air-FAR](https://github.com/Bottle101/Air-FAR) |
| SUPER | `sitl_super_planner_od.launch` | 三维激光雷达 | [SUPER](https://github.com/hku-mars/SUPER) |

多项式轨迹生成器本身不具备避障能力。使用感知规划器时，传感器话题、坐标系、地图和规划参数必须与所选仿真机型一致。

## 仓库结构

```text
OpenDrone/
|-- controller/                    控制器 ROS 包
|-- planner/                       轨迹生成器与在线规划器
|-- opendrone/
|   |-- launch/                    集成启动文件
|   |   `-- benchmarks/            面向评测的 launch 包装层
|   |-- msg/                       统一规划消息
|   |-- rviz/                      RViz 配置
|   |-- scripts/
|   |   `-- flight_eval/           实验与分析框架
|   |-- sitl_config/               PX4/Gazebo 资源（Git 子模块）
|   `-- src/                       基础节点与示例
|-- utils/                         公共消息、库和插件
|-- shell/                         配置、录包和降落脚本
|-- cmake/Modules/                 CMake 依赖查找模块
`-- .github/workflows/             ROS Noetic 构建验证
```

## 支持环境

当前持续集成验证的目标环境如下：

| 组件 | 已验证目标 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| 仿真器 | PX4 SITL + Gazebo Classic |
| 构建系统 | catkin tools |

部分上游包可能也能在 ROS Melodic 下构建，但当前仓库的 CI 不验证该组合。

## 安装

### 1. 准备 PX4 与 ROS

安装 ROS Noetic、MAVROS，以及提供 Gazebo Classic SITL 的 PX4-Autopilot 版本。集成 OpenDrone 前，请先确认 PX4 的标准启动命令能够正常运行：

```bash
roslaunch px4 mavros_posix_sitl.launch
```

基础环境配置可参考 [PX4 ROS 1 / MAVROS 文档](https://docs.px4.io/main/zh/ros/mavros_installation)。

### 2. 创建 catkin 工作空间并递归克隆

```bash
sudo apt update
sudo apt install -y python3-catkin-tools python3-rosdep git

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/Tfly6/OpenDrone.git
```

如果克隆时没有使用 `--recursive`，请补充初始化子模块：

```bash
cd ~/catkin_ws/src/OpenDrone
git submodule update --init --recursive
```

必需的子模块为 `utils/catkin_simple`、`utils/mav_comm` 和 `opendrone/sitl_config`。

### 3. 安装系统与 ROS 依赖

```bash
sudo apt install -y \
  libgoogle-glog-dev libgflags-dev libeigen3-dev \
  libarmadillo-dev liblapacke-dev liblapack-dev libopenblas-dev \
  libdw-dev libopencv-dev libyaml-cpp-dev

sudo rosdep init || true
rosdep update
cd ~/catkin_ws
rosdep install --from-paths src/OpenDrone --ignore-src --rosdistro noetic -r -y
```

多项式轨迹包需要 NLopt 2.7.1：

```bash
cd /tmp
git clone --branch v2.7.1 --depth 1 https://github.com/stevengj/nlopt.git
cmake -S nlopt -B nlopt/build
cmake --build nlopt/build -j"$(nproc)"
sudo cmake --install nlopt/build
```

### 4. 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin init
catkin build
source devel/setup.bash
```

MPC 相关包会显著增加编译时间。不需要 MPC 时可跳过：

```bash
catkin config --skiplist mav_disturbance_observer mav_linear_mpc mav_nonlinear_mpc
catkin build
```

### 5. 部署仿真资源

完成 PX4 SITL 构建后，将 OpenDrone 的模型、world、launch 文件和已有插件复制到自动检测到的 PX4 Gazebo Classic 目录：

```bash
cd ~/catkin_ws/src/OpenDrone
./shell/gazeboSetup.bash /absolute/path/to/PX4-Autopilot
```

脚本兼容 `Tools/sitl_gazebo` 和 `Tools/simulation/gazebo-classic/sitl_gazebo-classic` 两种目录布局。它会向 PX4 目录复制文件，因此请先将本地 PX4 定制内容纳入版本控制或做好备份。

## 快速开始

以下每个终端都需要加载 ROS 和工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### PX4 基础 Offboard 示例

终端 1：

```bash
roslaunch px4 mavros_posix_sitl.launch
```

终端 2：

```bash
rosrun opendrone offb_node
```

该示例会进入 Offboard 模式、解锁、上升到约 2 m 并保持位置。

### 几何控制器与多项式轨迹

在终端 1 启动 PX4 SITL，然后运行控制器：

```bash
roslaunch opendrone sitl_se3_hopf.launch
```

在另一个终端启动轨迹生成器：

```bash
roslaunch opendrone sitl_mav_trajectory_planner.launch
```

### EGO-Planner 与深度相机

在终端 1 启动带深度相机的 PX4 机型与场景：

```bash
roslaunch px4 outdoor_depth_camera.launch
```

随后分别启动控制器与规划器：

```bash
roslaunch opendrone sitl_se3_hopf.launch
```

```bash
roslaunch opendrone sitl_ego_planner.launch
```

使用 Mid-360 示例前，请先按照 [Mid360_px4_sim_plugin](https://github.com/Tfly6/Mid360_px4_sim_plugin) 安装仿真模型，然后组合 `outdoor_mid360.launch` 与名称包含 `*_mid360.launch` 的规划入口。

触发降落：

```bash
cd ~/catkin_ws/src/OpenDrone
./shell/trigger_land.sh
```

## 评测与基准实验

OpenDrone 内置 `flight_eval`，可执行控制、轨迹生成和在线规划任务，并支持录制 rosbag、重新计算指标、生成图表以及运行 YAML 实验矩阵。

```bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH

python3 -m flight_eval list-controllers
python3 -m flight_eval list-planners
python3 -m flight_eval list-tasks
```

请从[飞行评测 README](opendrone/scripts/flight_eval/README_CN.md) 开始阅读；其中的用户指南与开发者指南分别说明环境准备、实验操作、任务语义、指标和外部算法 manifest。评测包装 launch 直接按类别组织在 `opendrone/launch/benchmarks/` 下。

## 接入新算法

接入规划器时，可以直接向 `/planner/output` 发布 `opendrone/PlannerOutput`，也可以在 `planner_adapter.py` 中增加转换路径。接口需要满足以下约定：

- `header.frame_id` 使用与 MAVROS 里程计一致的局部 ENU 坐标系。
- 为每个实际填充的导数或偏航字段设置 `valid_mask`。
- 同一条活动轨迹的不同窗口保持相同的 `trajectory_id`。
- `trajectory_status` 只表示局部轨迹生命周期，不能代替航点或整项任务完成状态。
- 任务需要时，通过 `/planner/mission_state` 单独发布任务进度。

如需接入评测框架，请在 `opendrone/launch/benchmarks/` 下增加包装 launch，并添加算法 manifest 条目。详细契约见[飞行评测开发者指南](opendrone/scripts/flight_eval/docs/developer-guide_CN.md)。

## 常见问题

- **配置阶段提示 ROS 包缺失：** 检查 `git submodule status`，每个必需条目都应显示已检出的提交，不能以 `-` 开头。
- **找不到 PX4 模型或 world：** 先构建 PX4 SITL，再使用正确的 PX4 根目录重新运行 `shell/gazeboSetup.bash`。
- **MAVROS 无法连接：** 确认 FCU URL 以及 UDP/TCP 端口与 PX4、Gazebo 配置一致。
- **规划器收不到传感器数据：** 检查 launch 参数和 RViz 中的话题名称与 TF 坐标系。
- **完整编译耗时过长：** 使用上文的 catkin skiplist，只启用当前实验所需的软件包。

## 上游项目与许可证

OpenDrone 集成并改编了算法表中所列项目的成果。相关源文件和依赖可能采用各自的许可证与引用要求；在重新分发或发表研究成果前，请查阅对应目录和上游仓库。

OpenDrone 顶层自有代码采用 [MIT License](LICENSE)。
