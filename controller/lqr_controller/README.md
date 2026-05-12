# lqr_controller

`lqr_controller` 是一个基于 ROS 的状态依赖线性二次调节器（State Dependent LQR）控制节点，旨在实现高精度的轨迹跟踪控制。

## 1. 功能说明

- 实现状态依赖的 LQR 控制，支持欧拉角和四元数两种姿态表示。
- 提供高精度轨迹跟踪能力，适用于复杂飞行任务。
- 支持参数化配置，便于调试和优化。

## 2. 订阅与发布

### 订阅

- `/mavros/state` (`mavros_msgs/State`)：无人机飞控状态。
- `/mavros/local_position/pose` (`geometry_msgs/PoseStamped`)：无人机当前位置。
- `/mavros/local_position/velocity_local` (`geometry_msgs/TwistStamped`)：无人机当前速度。
- `/command/trajectory` (`trajectory_msgs/MultiDOFJointTrajectory`)：目标轨迹。

### 发布

- `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`)：位置目标。
- `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`)：姿态目标。

## 3. 关键参数

节点私有参数（`~` 命名空间）：

- `enable_sim`：是否仿真模式（`true` 时自动尝试切 Offboard 和解锁）。
- `takeoff_height`：起飞高度，默认 `2.0`。
- `lqr_mode`：控制模式
  - `0` -> 欧拉角模式
  - `1` -> 四元数模式

## 4. 控制理论与工程实现要点

本节点的控制目标可概括为：通过 LQR 控制器生成最优控制输入 $u(t)$，使无人机状态 $x(t)$ 跟踪期望状态 $x_d(t)$，并最小化以下代价函数：

$$
J = \int_0^\infty \left[ (x - x_d)^T Q (x - x_d) + u^T R u \right] dt
$$

- $Q$：状态误差权重矩阵，决定状态偏差的惩罚程度。
- $R$：控制输入权重矩阵，决定控制能量的惩罚程度。

### 4.1 状态依赖 LQR 的优势

- 考虑系统的非线性特性，通过在线更新线性化模型提升控制精度。
- 相比传统 PID 控制，LQR 能够显式优化控制能量，避免过度控制。

### 4.2 离散实现与参数调优

控制器在离散周期 $\Delta t$ 下运行，离散形式为：

$$
K = \mathrm{dlqr}(A, B, Q, R)
$$

其中 $A$ 和 $B$ 为线性化后的系统矩阵，$K$ 为反馈增益矩阵。

调参建议：

- 增大 $Q$ 的对角元素可提升轨迹跟踪精度，但可能导致控制输入增大。
- 增大 $R$ 的对角元素可减少控制能量，但可能降低跟踪性能。

## 5. 使用方法

### 5.1 编译

在工作空间根目录执行：

```bash
cd ~/catkin_ws
catkin build lqr_controller
source devel/setup.bash
```

### 5.2 启动节点

运行主控制器并加载参数文件：

```bash
roslaunch lqr_controller lqr.launch
```

### 5.3 调参建议

- 根据任务需求调整 `lqr_cost/Q` 和 `lqr_cost/R`。
- 优先在仿真环境中验证参数效果，再应用于实机测试。

## . 参考
[1] Foehn, Philipp & Scaramuzza, Davide. (2018). Onboard State Dependent LQR for Agile Quadrotors. 10.1109/ICRA.2018.8460885.