# se3_hopf

`se3_hopf` 是一个基于 ROS + MAVROS 的四旋翼几何控制节点，核心在于：

- 使用 SE(3) 几何控制框架进行位置/速度/姿态/角速度的层级误差反馈
- 通过 Hopf fibration 从平坦输出构造期望姿态与角速度，降低传统构型在特殊姿态附近的奇异性问题
- 输出姿态与油门命令到 PX4 Offboard 接口

节点内置基础飞行状态机流程：连接等待 -> Offboard 准备 -> 任务执行 -> 降落/着陆。

## 1. 功能说明

- 支持接收轨迹指令并执行跟踪（`/command/trajectory`）
- 支持仿真模式下自动 Offboard 与自动解锁
- 提供地理围栏越界保护，越界后触发降落
- 提供 `/land` 服务触发降落
- 支持 `dynamic_reconfigure` 在线调参（位置/速度/加速度/姿态/角速度环）

## 2. 订阅与发布

### 订阅

- `/mavros/state` (`mavros_msgs/State`)
- `/mavros/local_position/odom` (`nav_msgs/Odometry`)
- `/mavros/imu/data` (`sensor_msgs/Imu`)
- `/command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)

### 发布

- `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`)
- `/mavros/setpoint_position/local` (`geometry_msgs/PoseStamped`)：Offboard 准备阶段用于位置保持/起飞前置

### 服务

- `/land` (`std_srvs/SetBool`)：调用后切换 `AUTO.LAND`

## 3. 关键参数

节点私有参数（`~` 命名空间）：

- `enable_sim`：是否仿真模式（`true` 时自动尝试 Offboard/解锁）
- `takeoff_height`：起飞目标高度（默认 `2.0`）
- `geo_fence/x`、`geo_fence/y`、`geo_fence/z`：地理围栏边界（越界保护）

### dynamic_reconfigure 参数（`cfg/tune.cfg`）

- P 增益
  - `kp_px/py/pz`
  - `kp_vx/vy/vz`
  - `kp_ax/ay/az`
  - `kp_qx/qy/qz`
  - `kp_wx/wy/wz`
- D 增益
  - `kd_px/py/pz`
  - `kd_vx/vy/vz`
  - `kd_ax/ay/az`
  - `kd_qx/qy/qz`
  - `kd_wx/wy/wz`
- 误差限幅
  - `limit_err_p`、`limit_err_v`、`limit_err_a`
  - `limit_d_err_p`、`limit_d_err_v`、`limit_d_err_a`

## 4. 控制理论与工程实现要点

### 4.1 分层误差反馈结构

该控制器在代码中体现为层级误差反馈：

1. 位置误差 $e_p=p-p_d$ 修正速度参考
2. 速度误差 $e_v=v-v_d$ 修正加速度参考
3. 加速度误差 $e_a=a-a_d$ 进一步修正平坦输出
4. 将平坦输出映射为期望姿态/角速度，再结合姿态与角速度误差得到最终机体角速度与推力命令

对应实现直觉可写为：

$$
v_d \leftarrow v_d - K_{p,p}e_p - K_{d,p}\dot e_p
$$

$$
a_d \leftarrow a_d - K_{p,v}e_v - K_{d,v}\dot e_v + g e_3
$$

其中加速度项再进入姿态构造与角速度求解。

### 4.2 Hopf fibration 的工程意义

传统由加速度方向构造姿态的方法，在某些姿态附近会出现数值不稳定或奇异性放大。`se3_hopf` 使用 Hopf fibration 进行四元数构造，在姿态参数化上更平滑，适合高机动轨迹与快速姿态变化场景。

### 4.3 推力归一化与在线估计

控制输出推力使用归一化形式：

$$
T_{cmd}=\frac{T}{T_a}
$$

其中 $T_a$ 是归一化常数（与机体推重关系相关）。控制器内部提供 `estimateTa(...)` 的在线估计机制，用于减小模型误差对油门映射的影响。

### 4.4 限幅策略为什么重要

`limit_err_*` 与 `limit_d_err_*` 会直接约束误差和误差变化率，目的包括：

- 避免瞬时大误差导致控制量突变
- 降低噪声放大对姿态环的冲击
- 提升真实机体上的稳定性与可调性

## 5. 使用方法

## 5.1 编译

在工作空间根目录执行：

```bash
cd ~/catkin_ws
catkin build se3_hopf
source devel/setup.bash
```

## 5.2 启动控制节点

```bash
rosrun se3_hopf se3_hopf_node _enable_sim:=true _takeoff_height:=2.0
```

## 5.3 配合 PX4 SITL 与在线调参

先启动 PX4 + MAVROS（示例）：

```bash
roslaunch px4 mavros_posix_sitl.launch
```

再启动控制节点，并打开动态调参：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

轨迹可由上层节点发布到 `/command/trajectory`。

## 6. 调参建议

- 先调平移链路：`kp_p* -> kp_v* -> kp_a*`
- 再调姿态与角速度链路：`kp_q* -> kp_w*`
- D 项从小到大逐步加：`kd_*` 主要用于抑制振荡，不宜一次给大
- 若出现高频抖动：先减小 `kp_w*`/`kd_w*`，同时收紧 `limit_d_err_*`
- 若跟踪“慢且肉”：优先提高 `kp_p*` 与 `kp_v*`，再看 `kp_a*`
- 若推力响应偏差明显：检查 `estimateTa` 收敛与悬停油门设置

## 7. 参考文献

- M. Faessler, A. Franchi, and D. Scaramuzza, Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories, IEEE RA-L, 2018.
- M. Watterson and V. Kumar, Control of Quadrotors Using the Hopf Fibration on SO(3), Robotics Research, 2020.
- T. Lee, M. Leok, and N. H. McClamroch, Geometric Tracking Control of a Quadrotor UAV on SE(3), CDC, 2010.
