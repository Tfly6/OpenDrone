# OpenDrone
基于ROS1的PX4无人机仿真，参考了多个开源项目，并把他们集成在PX4 SITL中，其中：

**控制器（controller文件夹）**

- **geometric_controller**：参考了[Jaeyoung-Lim/mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers) 项目，包含了多个方法。

  启动：

  ```bash
  roslaunch opendrone sitl_geometric_controller.launch # 默认Nonlinear GeometricControl
  # 降落
  # ./shell/trigger_land.sh
  ```

- **se3_controller**：参考了[HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller) 项目，包含了多个方法。

  启动：

  ```bash
  roslaunch opendrone sitl_se3_controller.launch # 默认 Hopf Fibration on SO(3)
  # 降落
  # ./shell/trigger_land.sh
  ```

- **pid_controller**：里面包含了简单pid（仅供学习）和级联pid。

  启动：

  ```bash
  roslaunch opendrone sitl_pid_controller.launch # 默认 cascade pid
  # 降落
  # ./shell/trigger_land.sh
  ```
  
  

**规划器（planner）**

- **polynomial trajectory generation**：参考了[ethz-asl/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) 项目，是一个基于优化方法的多项式轨迹生成，并不能实时规划与避障。

  启动：

  ```bash
  roslaunch opendrone sitl_mav_trajectory_planner.launch
  ```

- **ego_planner**：参考了[ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 项目，需要带深度相机的无人机。

  启动：

  ```bash
  roslaunch opendrone sitl_camera.launch # 相机坐标转换
  roslaunch opendrone sitl_ego_planner.launch
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

- **依赖** 

```bash
sudo apt install libgoogle-glog-dev libgflags-dev libeigen3-dev libarmadillo-dev
sudo apt install ros-$ROS_DISTRO-pcl-ros
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
https://github.com/Tfly6/OpenDrone.git
cd ~/catkin_ws
catkin build
```
## 3. 运行
以官方案例为例（ `opendrone/src/basic_example` 目录下是一些基础案例，不包含控制器和规划器）
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

## 参考

[1] Lee, Taeyoung, Melvin Leoky, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." Decision and Control (CDC), 2010 49th IEEE Conference on. IEEE, 2010.

[2] Faessler, Matthias, Antonio Franchi, and Davide Scaramuzza. "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories." IEEE Robot. Autom. Lett 3.2 (2018): 620-626.

[3] D. Mellinger and V. Kumar, “Minimum snap trajectory generation and control for quadrotors,” in Proc. of the IEEE Intl. Conf. on Robot. and Autom. (ICRA), Shanghai, China, May 2011, pp. 2520–2525.

[4] X. Zhou, Z. Wang, H. Ye, C. Xu, and F. Gao, “EGO-Planner: An ESDFfree gradient-based local planner for quadrotors,” IEEE Robotics and Automation Letters, vol. 6, no. 2, pp. 478–485, 2021.