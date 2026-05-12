# pid_controller

`pid_controller` 是一个基于 ROS + MAVROS 的无人机控制节点，支持两种控制模式：

- `SIMPLE_PID`（`pid_type=0`）：位置 PID，输出位置设定值到 `/mavros/setpoint_position/local`
- `CASCADE_PID`（`pid_type=1`）：串级 PID（位置环 + 速度环），输出姿态/油门到 `/mavros/setpoint_raw/attitude`

节点包含基础状态机流程：连接等待 -> Offboard -> 解锁 -> 起飞 -> 任务执行 -> 降落/应急。

## 1. 功能说明

- 自动管理 Offboard 与解锁（仿真模式下）
- 接收轨迹指令并跟踪目标点
- 提供地理围栏（`geo_fence`）越界保护，越界进入应急状态
- 提供 `/land` 服务触发降落
- 支持 `dynamic_reconfigure` 在线调整串级 PID 参数

## 2. 订阅与发布

### 订阅

- `/mavros/state` (`mavros_msgs/State`)
- `/mavros/local_position/pose` (`geometry_msgs/PoseStamped`)
- `/mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`)
- `/mavros/imu/data_raw` (`sensor_msgs/Imu`)
- `/command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)
- `/waypoint_generator/waypoints` (`nav_msgs/Path`)

### 发布

- `/mavros/setpoint_position/local` (`geometry_msgs/PoseStamped`)
- `/mavros/setpoint_velocity/cmd_vel_unstamped` (`geometry_msgs/Twist`)
- `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`)
- `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`)

### 服务

- `/land` (`std_srvs/SetBool`)：调用后切换为 `AUTO.LAND`

## 3. 关键参数

节点私有参数（`~` 命名空间）：

- `enable_sim`：是否仿真模式（`true` 时自动尝试切 Offboard 和解锁）
- `takeoff_height`：起飞高度，默认 `2.0`
- `pid_type`：控制器类型
  - `0` -> `SIMPLE_PID`
  - `1` -> `CASCADE_PID`
- `geo_fence/x`、`geo_fence/y`、`geo_fence/z`：围栏限制

### SIMPLE_PID 参数（`~Pos_pid/*`）

- 增益：`Kp_x/y/z`、`Ki_x/y/z`、`Kd_x/y/z`
- 限幅：`px_error_max`、`py_error_max`、`pz_error_max`

### CASCADE_PID 参数（`~cascade_pid/*`）

- 位置环：`Kp_px/py/pz`
- 速度环：`Kp_vx/vy/vz`、`Ki_vx/vy/vz`、`Kd_vx/vy/vz`
- 限幅：`maxVel`、`maxAcc`、`pos_err_max`、`vel_err_max`、`vel_integral_max`

### dynamic_reconfigure 参数

通过 `rqt_reconfigure` 可在线调整（见 `cfg/PidController.cfg`）：

- `kp_px/py/pz`
- `kp_vx/vy/vz`
- `ki_vx/vy/vz`
- `kd_vx/vy/vz`
- `pos_error_max`、`vel_error_max`、`vel_integral_max`

## 4. 控制理论与工程实现要点

本节点的控制目标可概括为：让无人机状态 $x(t)$ 跟踪期望状态 $x_d(t)$，并在外部扰动（风、模型误差、传感器噪声）下保持稳定和可控。

### 4.1 PID 基本形式

定义误差：$e(t)=x_d(t)-x(t)$。

连续时间 PID 为：

$$
u(t)=K_p e(t)+K_i\int_0^t e(\tau)d\tau+K_d\frac{de(t)}{dt}
$$

- 比例项 $K_p$：决定“当前误差”响应强度，过小会拖沓，过大易震荡。
- 积分项 $K_i$：消除稳态误差，但过大容易积分饱和（windup）。
- 微分项 $K_d$：提供阻尼，抑制超调，对噪声较敏感。

### 4.2 为何采用串级 PID

`CASCADE_PID` 将控制分为两层：

- 外环（位置环）：根据位置误差生成速度期望。
- 内环（速度环）：根据速度误差生成加速度/姿态/推力命令。

这种结构相比单环位置 PID 的优势：

- 更符合飞控执行链路（位置 -> 速度 -> 姿态/推力）。
- 内环更快、外环更慢，便于分频设计，稳定性更好。
- 调参可分层进行，工程上更直观。

### 4.3 离散实现直觉（与参数限幅的关系）

控制器在离散周期 $\Delta t$ 下运行，可理解为：

$$
e_k=x_{d,k}-x_k
$$

$$
I_k=\mathrm{clip}(I_{k-1}+e_k\Delta t,\ -I_{max},\ I_{max})
$$

$$
D_k=\frac{e_k-e_{k-1}}{\Delta t}
$$

$$
u_k=K_p e_k+K_i I_k+K_d D_k
$$

其中 `pos_err_max`、`vel_err_max`、`vel_integral_max`、`maxVel`、`maxAcc` 的作用分别是：

- 误差限幅：避免异常大误差直接打满控制量。
- 积分限幅：抑制 windup，减少“放开后反向过冲”。
- 速度/加速度限幅：满足执行器与飞行包线约束，提升安全性。

### 4.4 简化调参逻辑（为什么推荐当前顺序）

- 先调位置环 `kp_p*`：决定轨迹跟随“拉回”力度。
- 再调速度环 `kp_v*`：决定速度响应快慢与阻尼。
- 最后加小积分 `ki_v*`（常先从 `z` 轴开始）：消除稳态偏差。

若出现问题，可按机理排查：

- 高频抖动/噪声放大：适当降 `kd_v*` 或改善速度估计滤波。
- 低频来回摆动：通常 `kp_p*` 或 `kp_v*` 偏大，或环间带宽分离不足。
- 高度慢慢漂：`ki_vz` 可能偏小；若上下抽动，`ki_vz` 可能偏大。

## 5. 使用方法

## 5.1 编译

在工作空间根目录执行：

```bash
cd ~/catkin_ws
catkin build pid_controller
source devel/setup.bash
```

## 5.2 直接启动节点

```bash
rosrun pid_controller pid_controller_node _enable_sim:=true _takeoff_height:=2.0 _pid_type:=1
```

## 5.3 使用 OpenDrone 的 SITL 启动文件

你当前工程中可直接使用：

```bash
roslaunch opendrone sitl_pid_controller.launch
```

该 launch 已设置：

- `pid_type=1`（串级 PID）
- `cascade_pid/maxAcc` 可通过 `max_acc` 参数传入
- 启动 `rqt_reconfigure` 便于在线调参

示例（修改起飞高度和最大加速度）：

```bash
roslaunch opendrone sitl_pid_controller.launch takeoff_height:=3.0 max_acc:=5.0
```

## 6. 调参建议

- 先固定 `pid_type=1`，优先调 `kp_p*`（位置环）保证响应速度
- 再调速度环 `kp_v*`，最后加少量 `ki_vz` 消除高度静差
- 若出现明显震荡，先降低 `kp_v*` 或增大误差限幅约束
- 推力饱和时优先检查 `maxAcc` 与速度环增益
