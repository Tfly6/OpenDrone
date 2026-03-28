# se3_controller
[原README](./README_old.md)
`se3_controller` 是一个基于 ROS + MAVROS 的四旋翼 SE(3) 控制节点，面向姿态/角速度 + 推力控制链路。

节点集成了基础飞行状态机流程：连接等待 -> Offboard -> 解锁 -> 任务执行 -> 降落。

## 1. 功能说明

- 自动管理 Offboard 与解锁（`enable_sim=true` 时）
- 接收期望轨迹并执行 SE(3) 跟踪控制
- 发布姿态目标到 PX4 原始接口（`/mavros/setpoint_raw/attitude`）
- 提供地理围栏（`geo_fence`）越界保护，越界触发降落
- 提供 `/land` 服务触发降落
- 支持 `dynamic_reconfigure` 在线调整控制增益与目标姿态

## 2. 订阅与发布

### 订阅

- `/mavros/state` (`mavros_msgs/State`)
- `/mavros/local_position/odom` (`nav_msgs/Odometry`)
- `/mavros/imu/data` (`sensor_msgs/Imu`)
- `/desire_odom` (`nav_msgs/Odometry`)
- `/command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)

### 发布

- `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`)
- `/mavros/setpoint_position/local` (`geometry_msgs/PoseStamped`)
- `/desire_odom_pub` (`nav_msgs/Odometry`)

### 服务

- `/land` (`std_srvs/SetBool`)：调用后切换为 `AUTO.LAND`

## 3. 关键参数

节点私有参数（`~` 命名空间）：

- `enable_sim`：是否仿真模式（`true` 时自动尝试切换 `OFFBOARD` 并解锁）
- `takeoff_height`：起飞高度，默认 `2.0`
- `geo_fence/x`、`geo_fence/y`、`geo_fence/z`：围栏限制

在 OpenDrone SITL 启动文件中默认值为：

- `enable_sim=true`
- `takeoff_height=2.0`
- `geo_fence/x=50.0`
- `geo_fence/y=50.0`
- `geo_fence/z=6.0`

## 4. dynamic_reconfigure 参数

通过 `rqt_reconfigure` 可在线调整（见 `cfg/tune.cfg`）：

- 比例项：`kp_p*`、`kp_v*`、`kp_a*`、`kp_q*`、`kp_w*`
- 微分项：`kd_p*`、`kd_v*`、`kd_a*`、`kd_q*`、`kd_w*`
- 误差限幅：`limit_err_p/v/a`、`limit_d_err_p/v/a`
- 目标指令：`desire_px/py/pz`、`desire_yaw`

## 5. 使用方法

### 5.1 编译

在工作空间根目录执行：

```bash
cd ~/test1_ws
catkin build se3_controller
source devel/setup.bash
```

### 5.2 直接启动节点

```bash
rosrun se3_controller se3_controller_node _enable_sim:=true _takeoff_height:=2.0
```

### 5.3 使用 OpenDrone 的 SITL 启动文件

你当前工程中可直接使用：

```bash
roslaunch opendrone sitl_se3_controller.launch
```

该 launch 已设置：

- 启动 `se3_controller_node`
- 默认仿真模式与起飞高度参数
- 启动 `rqt_reconfigure` 便于在线调参

示例（按默认参数启动）：

```bash
roslaunch opendrone sitl_se3_controller.launch
```

如需修改起飞高度/围栏，可在启动前手动设置同名参数（或直接修改 launch 文件中的 `<param>`）：

```bash
rosparam set /se3_controller_node/takeoff_height 3.0
rosparam set /se3_controller_node/geo_fence/x 60.0
rosparam set /se3_controller_node/geo_fence/y 60.0
rosparam set /se3_controller_node/geo_fence/z 8.0
roslaunch opendrone sitl_se3_controller.launch
```

## 6. 调参建议

- 先调位置/速度相关比例项（`kp_p*`、`kp_v*`）保证轨迹可跟踪
- 再调姿态与角速度相关项（`kp_q*`、`kp_w*`）改善姿态响应
- 若高频抖动明显，优先降低对应通道的 `kp_*` 或增加适量 `kd_*`
- 若经常触发围栏降落，优先检查轨迹范围与 `geo_fence` 设置
