# se3_lee

`se3_lee` 是一个基于 ROS + MAVROS 的四旋翼几何轨迹跟踪控制节点，适用于 PX4 Offboard 模式。控制器核心包含：

- 几何姿态控制（SE(3) 框架）
- 位置/速度反馈 + 参考加速度前馈
- 旋翼阻力补偿（rotor drag compensation）
- 将期望加速度映射为姿态与机体系角速度，再输出推力与角速度命令

节点内置状态机流程：等待 home pose -> 任务执行 -> 降落/着陆（含越界应急分支）。

## 1. 功能说明

- 接收轨迹并进行跟踪（支持 `reference/setpoint` 与 `command/trajectory`）
- 支持仿真模式下自动 Offboard 与自动解锁
- 支持地理围栏保护（越界进入应急状态）
- 支持 `/land` 服务触发降落
- 支持 `dynamic_reconfigure` 在线调节关键增益与最大反馈加速度

## 2. 订阅与发布

### 订阅

- `reference/setpoint` (`geometry_msgs/TwistStamped`)
- `reference/yaw` (`std_msgs/Float32`)
- `command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)
- `mavros/state` (`mavros_msgs/State`)
- `mavros/local_position/pose` (`geometry_msgs/PoseStamped`)
- `mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`)

### 发布

- `command/bodyrate_command` (`mavros_msgs/AttitudeTarget`)
- `reference/pose` (`geometry_msgs/PoseStamped`)
- `mavros/setpoint_position/local` (`geometry_msgs/PoseStamped`)
- `mavros/companion_process/status` (`mavros_msgs/CompanionProcessStatus`)
- `se3_lee/state` (`std_msgs/Int8`)

### 服务

- `/land` (`std_srvs/SetBool`)
- `trigger_rlcontroller` (`std_srvs/SetBool`)

## 3. 关键参数

节点私有参数（`~` 命名空间）：

- 基础控制
  - `mavname`
  - `ctrl_mode`（默认 `ERROR_QUATERNION`）
  - `enable_sim`
  - `debug`
  - `velocity_yaw`
- 动力学与限制
  - `max_acc`
  - `max_vel`
  - `yaw_heading`
  - `drag_dx`、`drag_dy`、`drag_dz`
  - `attctrl_constant`
  - `normalizedthrust_constant`
  - `normalizedthrust_offset`
- 位置/速度反馈增益
  - `Kp_x`、`Kp_y`、`Kp_z`
  - `Kv_x`、`Kv_y`、`Kv_z`
- 任务与安全
  - `takeoff_height`
  - `geo_fence/x`、`geo_fence/y`、`geo_fence/z`
  - `posehistory_window`

### dynamic_reconfigure 参数（`cfg/GeometricController.cfg`）

- `max_acc`
- `Kp_x`、`Kp_y`、`Kp_z`
- `Kv_x`、`Kv_y`、`Kv_z`

## 4. 控制理论与工程实现要点

### 4.1 位置外环 + 姿态内环

该控制器采用“先算期望加速度，再映射到姿态/角速度”的结构。核心步骤为：

1. 由位置、速度误差计算反馈加速度
2. 叠加参考加速度前馈
3. 引入旋翼阻力补偿项
4. 从期望加速度和期望航向求期望姿态，再由姿态控制器输出角速度与推力

可写为：

$$
a_{fb}=K_p(p-p_d)+K_v(v-v_d)
$$

$$
a_{des}=a_{fb}+a_{ref}-a_{drag}-g
$$

其中阻力补偿在实现中体现为：

$$
a_{drag}=R_{ref} D R_{ref}^T v_{ref}
$$

### 4.2 期望姿态构造

利用 $a_{des}$ 与 yaw 构造机体系期望方向，再由旋转矩阵转换为四元数。这一过程保证推力方向与期望加速度一致，是几何控制稳定跟踪的关键。

### 4.3 推力归一化

推力输出经过线性归一化并裁剪到 `[0,1]`：

$$
T_{cmd}=\mathrm{clip}(k_t\cdot T + b_t, 0, 1)
$$

对应参数为 `normalizedthrust_constant` 与 `normalizedthrust_offset`。这两个参数与具体机体推重比、桨电配置强相关。

### 4.4 限制与安全机制

- `max_acc`：限制反馈加速度模长，防止大误差下命令过激
- `geo_fence/*`：越界切入应急分支
- 应急分支下输出回 home pose 的位置目标，便于人工接管或系统降落

## 5. 使用方法

## 5.1 编译

在工作空间根目录执行：

```bash
cd ~/catkin_ws
catkin build se3_lee
source devel/setup.bash
```

## 5.2 直接启动（SITL + MAVROS + 控制器）

推荐使用包内 launch：

```bash
roslaunch se3_lee sitl_trajectory_track_circle.launch
```

或：

```bash
roslaunch se3_lee trajectory_controller.launch
```

上述 launch 会完成：

- 启动 `se3_lee_node`
- 将 `command/bodyrate_command` 重映射到 `/mavros/setpoint_raw/attitude`
- 启动 MAVROS/PX4（对应 launch 场景）
- 可选启动 `rqt_reconfigure` 进行在线调参

## 5.3 单独启动节点（已有外部 MAVROS/PX4）

```bash
rosrun se3_lee se3_lee_node _enable_sim:=true _takeoff_height:=2.0
```

## 6. 调参建议

- 先调平移外环：`Kp_*` 决定“拉回力度”，`Kv_*` 决定阻尼
- 抖动明显时先降 `Kv_*` 与 `max_acc`，确认不是推力映射过激
- 跟踪偏慢时先增 `Kp_*`，再配合 `Kv_*` 抑制过冲
- 高速轨迹误差大时检查 `drag_d*` 与 `normalizedthrust_*` 是否匹配机体
- 若航向出现不稳定，先固定 `velocity_yaw=false` 并单独调 yaw 参考

## 7. 参考文献

- T. Lee, M. Leok, and N. H. McClamroch, Geometric Tracking Control of a Quadrotor UAV on SE(3), CDC, 2010.
- M. Faessler, A. Franchi, and D. Scaramuzza, Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories, IEEE RA-L, 2018.
