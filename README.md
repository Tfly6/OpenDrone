# OpenDrone
基于ROS1的PX4无人机仿真
> 支持 Ubuntu 18.04 ROS Melodic PX4 v1.13.2 、Ubuntu 20.04 ROS Noetic
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
sudo apt-get install ros-`rosversion -d`-ompl
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
以官方案例为例（更多例子可以查看 `opendrone/src` 目录下的源文件）
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