# OpenDrone
基于ROS1的PX4无人机仿真，参考了多个开源项目，并把他们集成在PX4 SITL中，其中：

## 项目目录树

```text
OpenDrone/
├── cmake/                # CMake 模块与依赖查找脚本，用于兼容 glog、gflags、OpenBLAS 等第三方库
│   └── Modules/
├── controller/           # 控制器集合，统一封装为 ROS 包，便于在 PX4 SITL 中切换不同控制策略
├── opendrone/            # 主 ROS 功能包
│   ├── config/           # 参数配置
│   ├── include/          # 公共头文件
│   ├── launch/           # 一键启动 SITL、控制器、规划器和测试流程的 launch 文件
│   ├── rviz/             # RViz 配置
│   ├── scripts/          # Python 辅助脚本，例如消息转换、相机位姿发布、点云处理
│   ├── sitl_config/      # PX4/Gazebo SITL 所需的模型、世界、插件列表与仿真配置
│   └── src/              # 基础示例和辅助工具实现
├── planner/              # 规划器集合
├── shell/                # 常用脚本
├── utils/                # 公共依赖、消息定义、数学工具、可视化和配套基础库，供控制器与规划器复用
├── CMakeLists_Template.txt
├── LICENSE
└── README.md
```

> 补充说明：仓库内大多数 ROS 包都遵循类似的目录组织方式，例如 **cfg/** 用于动态参数配置，**include/** 用于头文件，**launch/** 用于启动文件，**src/** 用于源码实现，**test/** 用于测试或示例验证。

**控制器（controller文件夹）**

- **geometric_controller**：参考了[Jaeyoung-Lim/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers) 项目，具体看 [README.md](./controller/geometric_controller/README.md)。

  启动：

  ```bash
  roslaunch opendrone sitl_geometric_controller.launch # 默认Nonlinear GeometricControl
  # 降落
  # ./shell/trigger_land.sh
  ```

- **se3_controller**：参考了[HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller) 项目，具体看 [README.md](./controller/se3_controller/README.md)。（**推荐使用**）

  启动：

  ```bash
  roslaunch opendrone sitl_se3_controller.launch # 默认 Hopf Fibration on SO(3)
  # 降落
  # ./shell/trigger_land.sh
  ```

- **pid_controller**：里面包含了简单pid（仅供学习）和级联pid，具体看 [README.md](./controller/pid_controller/README.md)。

  启动：

  ```bash
  roslaunch opendrone sitl_pid_controller.launch # 默认 cascade pid
  # 降落
  # ./shell/trigger_land.sh
  ```
  
- **lqr_controller** : 参考了 [llanesc/lqr-tracking](https://github.com/llanesc/lqr-tracking) 项目，是一个简单的 lqr 控制器。

  启动：

  ```bash
  roslaunch opendrone sitl_lqr_controller.launch
  ```

- **mpc_controller**：参考了 [ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw) 项目，包含了线性mpc和非线性mpc，此控制器尚在实验中。
  启动：

  ```bash
  # 线性 mpc
  roslaunch opendrone sitl_mpc_controller.launch
  # 非线性 mpc
  roslaunch opendrone sitl_nmpc_controller.launch
  ```



**规划器（planner文件夹）**

- **polynomial trajectory generation**：参考了[ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) 项目，是一个基于优化方法的多项式轨迹生成，并不能实时规划与避障。

  启动：

  ```bash
  roslaunch opendrone sitl_mav_trajectory_planner.launch
  ```

- **fast_planner** : 参考了[HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) 项目，需要带深度相机的无人机。

  启动：

  ```bash
  roslaunch opendrone sitl_fast_planner.launch
  ```
  
- **ego_planner**：参考了[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 项目，需要带深度相机或3D激光雷达的无人机。（**推荐使用**）

  启动：

  ```bash
  roslaunch opendrone sitl_ego_planner.launch # 深度相机
  roslaunch opendrone sitl_ego_planner_mid360.launch # 激光雷达
  ```

- **ego_plannerV2** : 参考了 [ZJU-FAST-Lab/EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) 项目，需要带深度相机的无人机。

  启动：

  ```bash
  roslaunch opendrone sitl_ego_planner_v2.launch # 深度相机
  ```

- **airfar_planner** : 参考了 [Bottle101/Air-FAR](https://github.com/Bottle101/Air-FAR) 项目，需要带深度相机或3D激光雷达的无人机。此规划器尚在实验中。

  启动：

  ```bash
  roslaunch opendrone sitl_airfar_planner.launch # 深度相机
  roslaunch opendrone sitl_airfar_planner_mid360.launch # 激光雷达
  ```

  

> 支持 Ubuntu 18.04 ROS Melodic、Ubuntu 20.04 ROS Noetic
## 1. 准备
- **使用之前必须搭建** [PX4无人机仿真环境](https://blog.csdn.net/weixin_55944949/article/details/130895608?spm=1001.2014.3001.5501)

- **创建工作空间**
没有创建工作空间，可以执行下列代码，如果创建了可以跳过
```bash
sudo apt-get install python-catkin-tools python-rosinstall-generator -y

# For Ros Noetic use that:
# sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws && catkin init # 初始化工作空间
catkin build
```

- **安装依赖** 

```bash
sudo apt install libgoogle-glog-dev libgflags-dev libeigen3-dev libarmadillo-dev liblapacke-dev
sudo apt install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-laser-geometry ros-$ROS_DISTRO-tf2-sensor-msgs
```

**安装 NLopt 库** 

```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

## 2. 编译

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/Tfly6/OpenDrone.git
cd ~/catkin_ws
# 编译 mpc 相关的包需要十几分钟，所以如果不用 mpc 可以用下面命令把它列入编译黑名单（其他不用的包也是类似操作）
# catkin config --skiplist mav_control_interface mav_disturbance_observer mav_linear_mpc mav_nonlinear_mpc
catkin build
```
## 3. 运行

- **配置仿真** 

```bash
# model
# PX4 < v1.14
cp -r ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/models/* ${YOUR_PX4_PATH}/Tools/sitl_gazebo/models/
cp ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/worlds/* ${YOUR_PX4_PATH}/Tools/sitl_gazebo/worlds/

# PX4 >= v1.14
cp -r ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/models/* ${YOUR_PX4_PATH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
cp ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/worlds/* ${YOUR_PX4_PATH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
```

```bash
# launch
cp ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/outdoor_depth_camera.launch ${YOUR_PX4_PATH}/launch/
cp ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/outdoor_mid360.launch ${YOUR_PX4_PATH}/launch/
cp ~/catkin_ws/src/OpenDrone/opendrone/sitl_config/px4_config.yaml ${YOUR_PX4_PATH}/launch/
```



### 实例一：以官方案例为例

（ `opendrone/src/basic_example` 目录下是一些基础案例，不包含控制器和规划器）。

- 终端一：启动gazebo仿真
```bash
roslaunch px4 mavros_posix_sitl.launch
```
- 终端二：启动官方案例，起飞两米，并一直悬停
```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun opendrone offb_node
```

### 实例二：geometric_controller + polynomial trajectory generation

飞圆形。

- 终端一：启动gazebo仿真
```bash
roslaunch px4 mavros_posix_sitl.launch
```
- 终端二：启动 geometric_controller
```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_geometric_controller.launch
```
- 终端三：启动 polynomial trajectory generation
```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_mav_trajectory_planner.launch
```
演示视频 👇

[bilibili](https://www.bilibili.com/video/BV1EZTszKEtJ/?share_source=copy_web&vd_source=649164de6e400405dc9e781456725af7)


### 实例三：geometric_controller + ego_planner+深度相机
实时规划与避障

- 终端一：启动gazebo仿真

```bash
roslaunch px4 outdoor_depth_camera.launch # 用自己的也行
```
- 终端二：启动 geometric_controller
```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_geometric_controller.launch
```
- 终端三：启动 ego-planner
```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_ego_planner.launch
```
演示视频 👇

[bilibili](https://www.bilibili.com/video/BV17ZTszKEea/?share_source=copy_web&vd_source=649164de6e400405dc9e781456725af7)

### 实例四：geometric_controller + ego_planner+Mid360

实时规划与避障

- 根据下面仓库配置Mid360仿真 👇

[Tfly6/Mid360_px4_sim_plugin: Plugin for the simulation of the Livox Mid-360 in Gazebo](https://github.com/Tfly6/Mid360_px4_sim_plugin)

- 终端一：启动gazebo仿真

```bash
roslaunch px4 outdoor_mid360.launch # 用自己的也行
```

- 终端二：启动 geometric_controller

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_geometric_controller.launch
```

- 终端三：启动 ego-planner

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch opendrone sitl_ego_planner_mid360.launch
```

演示视频 👇

[Mid360 + ego-planner PX4无人机Gazebo仿真demo_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1dHFsz5EEh/?spm_id_from=333.1387.homepage.video_card.click&vd_source=d59e7d5891b69289e548bcfb7a4948a0)




## 相关论文

[1] Lee, Taeyoung, Melvin Leoky, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.

[2] Faessler, Matthias, Antonio Franchi, and Davide Scaramuzza. "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories." IEEE Robot. Autom. Lett 3.2 (2018): 620-626.

[3] D. Mellinger and V. Kumar, “Minimum snap trajectory generation and control for quadrotors,” in Proc. of the IEEE Intl. Conf. on Robot. and Autom. (ICRA), Shanghai, China, May 2011, pp. 2520–2525.

[4] X. Zhou, Z. Wang, H. Ye, C. Xu, and F. Gao, “EGO-Planner: An ESDFfree gradient-based local planner for quadrotors,” IEEE Robotics and Automation Letters, vol. 6, no. 2, pp. 478–485, 2021.

[5] Model Predictive Control for Trajectory Tracking of Unmanned Aerial Vehicles Using Robot Operating System. Mina Kamel, Thomas Stastny, Kostas Alexis and Roland Siegwart. Robot Operating System (ROS) The Complete Reference Volume 2. Springer 2017

[6] Linear vs Nonlinear MPC for Trajectory Tracking Applied to Rotary Wing Micro Aerial Vehicles. Mina Kamel, Michael Burri and Roland Siegwart. arXiv:1611.09240

[7] B. He, G. Chen, C. Fermuller, Y. Aloimonos and J. Zhang, "Air-FAR: Fast and Adaptable Routing for Aerial Navigation in Large-Scale Complex Unknown Environments," 2025 IEEE International Conference on Robotics and Automation (ICRA)

[8] Xin Zhou et al. ,Swarm of micro flying robots in the wild.*Sci. Robot.*7,eabm5954(2022)

[9] Foehn, Philipp & Scaramuzza, Davide. (2018). Onboard State Dependent LQR for Agile Quadrotors. 10.1109/ICRA.2018.8460885.

[10] B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight," in *IEEE Robotics and Automation Letters*, vol. 4, no. 4, pp. 3529-3536, Oct. 2019

