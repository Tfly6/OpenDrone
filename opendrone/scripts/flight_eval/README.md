# flight_eval - OpenDrone 飞行评估框架

`flight_eval` 用于自动化评估 OpenDrone 中的控制器、轨迹生成器和在线规划器。当前版本的核心原则是：

- `task` 定义实验语义，也就是“这次到底在测什么能力”
- manifest 可选 `tasks` 明确限制算法可以参与的任务
- 每个 `task` 固定定义一套事实指标，不提供评分、排名或总体评价
- 统一规划输出接口为 `/planner/output`
- controller / planner 的算法诊断 topic 由各自注册项声明，任务输入 topic 由 Task 声明

源码已经完成过多次重构；如果本文档和旧运行目录、旧对话记录不一致，以当前源码为准。

## 当前评估模型

```text
FlightRunner / BagAnalyzer
  ├── task: hover / analytic_* / discrete_* / plan_mission
  ├── algorithm: controller / planner manifest entry
  ├── metric_profile: hover_regulation / trajectory_tracking / trajectory_generation / online_mission
  ├── controller: se3_hopf / pid_controller / ...
  └── planner: none / mav_trajectory / rpg_trajectory / fast_planner_kino / ...
```

这些字段的分工是：

- `task` 决定任务目标、起点裁剪方式、指标集合主干
- manifest 的 `tasks` 只做可选白名单；省略表示不限制
- `metric_profile` 是 task 固定指标集合的稳定标识，不是可切换的报告视图

报告始终列出该任务的全部指标，并按 `group` 整理。每项指标声明 `source`、`phase` 和
`available`；框架不使用阈值给算法下结论。

## 任务指标集合

| metric_profile | 对应任务 |
|---|---|
| `hover_regulation` | `hover` |
| `trajectory_tracking` | `analytic_*` |
| `trajectory_generation` | `discrete_*` |
| `online_mission` | `plan_mission` |

## 快速开始

```bash
cd ~/src/test1_ws
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH

python3 -m flight_eval list-controllers
python3 -m flight_eval list-tasks
python3 -m flight_eval list-planners
```

## YAML 批次实验

`batch` 用 YAML 把 controller、planner、task 组合成可复现实验矩阵。Gazebo Classic 与 MAVROS
必须由使用者在同一个 launch 中先启动；`batch` 不创建、终止或重建它们。每条样本只启动独立的
PX4 SITL，因此 PX4 状态不会跨样本泄漏，也不会反复加载 world。
`batch_summary.json` 只记录样本清单和运行状态计数，不聚合指标、不评分，也不生成算法排名。

当 `transport.gazebo_px4: udp` 时，先生成与 YAML 指定 PX4 版本匹配的 UDP Iris 覆盖模型。
这一步不修改 PX4 的源码或 build 目录：

```bash
cd ~/src/test1_ws
source devel/setup.bash
cd ~/src/test1_ws/src/OpenDrone/opendrone/scripts
python3 -m flight_eval prepare-environment \
  --config ./flight_eval/examples/batch_example.yaml
```

然后在终端 A 中 source 输出的 `environment.env`，并执行输出的 `launch_command.sh`（或直接复制
终端打印出的命令），保持该 launch 运行：

```bash
source /tmp/flight_eval_udp_environment/environment.env
bash /tmp/flight_eval_udp_environment/launch_command.sh
```

该 launch 只包含 Gazebo、车辆模型和 MAVROS，刻意不包含 PX4。`prepare-environment` 从选定 PX4
的完整 Iris SDF 生成本次环境的模型覆盖，把 `mavlink_interface/use_tcp` 置为 `0`，并令
`mavlink_udp_port` 与 YAML 的 `transport.port` 一致。普通 Iris 直接使用该 SDF；Mid360/深度相机
模型仍通过优先级最高的 `model://iris` 覆盖自动使用它。`environment.launch` 强制把
`respawn_gazebo` 和 `respawn_mavros` 设为 `true`，这是每次 PX4 重启后自动恢复外部环境的前提；
YAML 若显式写这两个字段，也只能为 `true`。YAML 中的 `gazebo` / `mavros` / `transport` 必须与这个
已启动环境一致；同一 batch 不允许切换它们，需拆成多个 YAML 分别运行。

先用 dry-run 校验 YAML、算法任务限制和展开的样本数；它不会启动任何 ROS 进程：

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml \
  --dry-run
```

实际运行：

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml
```

任务执行期限、终点驻留时间和 `completion_time` 使用 ROS 时间。在 SITL
启用 `/use_sim_time` 时，它们因此表示仿真时间，不会因为 Gazebo
real-time factor 降低而提前超时。进程启动、连接、降落和 ROS 时钟停止推进
保护仍使用单调墙钟；ROS 时钟连续 30 秒不推进会以
`task_clock_stalled` 结束本次任务。

顶层 `name` 是必填的 batch 名称。未显式设置 `output_dir` 时，输出目录严格使用
`eval_runs/batches/<name>_<YYYYMMDD_HHMMSS>/`；experiment、controller 或 planner 名称不会
覆盖它。每条样本各有一个
`cases/<case>/` 目录，包含 bag、报告、核心图、`resolved_case.json` 和带 batch/world/sensor/PX4
上下文的 `run_metadata.json`；顶层 `batch_plan.json` 保存展开后的计划，`batch_summary.json` 会在
每条样本结束后增量更新。默认沿用单次 `run` 的自动分析，YAML 设置 `analysis: false` 或命令加
`--no-analyze` 可只录包。
每条样本还会使用独立的 `px4_rootfs/` 作为 PX4 工作目录，避免 `parameters.bson`、PX4 日志或
其他 SITL 状态从上一条样本继承。
`logs/environment_NNN/px4_<case>.log` 只保存 PX4 SITL 的启动、告警和退出标准输出，供环境
启动失败时诊断；飞行数据仍以 case 内的 bag 和 `px4_rootfs/log/**/*.ulg` 为准。相同 simulation
配置会复用同一 environment session，因此一个 `environment_NNN` 目录可能包含后续多个 case
的 PX4 控制台日志。PX4 以非交互模式启动，日志中不会记录 `pxh` 提示符重绘。

YAML 的最小结构如下（完整且可直接修改的例子见
[`examples/batch_example.yaml`](examples/batch_example.yaml)）：

```yaml
version: 1
manifests:                         # 可选；相对路径以本文件所在目录为基准
  - external_algorithm_manifest.yaml
name: example
defaults:
  simulation:
    gazebo:
      world: empty       # empty 或 opendrone/sitl_config/worlds 下的 world 名
      sensor: none       # none | mid360 | depth_camera；自定义 SDF 使用 gazebo.sdf
      respawn_gazebo: true  # 必须为 true；PX4 重启后自动刷新 Gazebo
      gui: true        # 默认 true；需要无界面运行时才设为 false
    mavros:
      fcu_url: udp://:14540@localhost:14557
      respawn_mavros: true  # 必须为 true；PX4 重启后自动刷新 MAVROS
    # 必填；不从 rospack 推断，因为 source devel/setup.bash 后该查找不可靠。
    px4:
      source_dir: /absolute/path/to/PX4-Autopilot
    transport:
      gazebo_px4: udp
      port: 14560
      environment_dir: /tmp/flight_eval_udp_environment
  run:
    takeoff_height: 2.0
    duration: 20
    auto_land: true
    max_collision_episodes: 3
    collision_episode_gap: 0.5
experiments:
  - name: repeat_and_matrix
    task: plan_mission
    controllers: [se3_hopf, mav_linear_mpc]
    planners: [ego_planner_mid360, airfar_planner_mid360]
    repeat: 3
    simulation:
      world: ego
      sensor: mid360
```

`controllers` / `planners` 与单数 `controller` / `planner` 都可用。列表会做笛卡尔积，再乘以
`repeat`；例如上例是 `2 × 2 × 3 = 12` 条。单项可写成 `{name, args}`；
experiment 的 `controller_args`、`planner_args` 分别作为该实验所有对应算法的公共 launch 参数。
`simulation` 可以放在 `defaults`，也可以在某一 experiment 覆盖；world/sensor 也允许写在
`simulation.gazebo` 下。可配置的 `run` 字段是 `task`、`takeoff_height`、
`duration`、`auto_land`、`extra_topics`、`max_collision_episodes` 和
`collision_episode_gap`。生成的 Iris 覆盖模型会自动加载轻量 Contact 插件：地面碰撞直接丢弃；
同一障碍物的连续 contact 心跳只累计为一个事件，间隔超过 `collision_episode_gap` 后再次接触才加一。
累计达到 `max_collision_episodes`（默认 3）时，runner 才安全终止任务。

### 挂载传感器

内置传感器直接设置 `gazebo.sensor: mid360` 或 `depth_camera`。它们使用 OpenDrone 的整机 wrapper：
wrapper 挂载传感器，内部 `model://iris` 自动解析为本次生成的 UDP Iris，所以传感器不会被覆盖掉。
自定义传感器则提供整机 `gazebo.sdf` 和其模型资源目录 `gazebo.model_paths`；资源目录会写入生成的
`environment.env`。该整机 SDF 应包含 `model://iris`，或自己直接定义 `mavlink_interface`。后者会被
复制到环境目录并自动改为 UDP；两者均不改原 SDF。改变 world、传感器、SDF 或 model_paths 后，必须
重新执行 `prepare-environment`，然后重启外部 Gazebo + MAVROS。

```yaml
simulation:
  gazebo:
    world: empty
    # 整机 wrapper 内应 include model://iris，再 include 你的传感器模型并用 joint 固定。
    sdf: ./models/iris_my_sensor/iris_my_sensor.sdf
    model_paths:
      - ./models
    respawn_gazebo: true
  mavros:
    respawn_mavros: true
```

batch 启动前会校验 UDP `environment.json` 与 YAML 的 PX4 路径和端口一致，并检查使用者启动的
`/gazebo/reset_world`、`/mavros/set_mode` 服务。Gazebo GUI 的默认值为 `true`。首条样本的 PX4
启动后，batch 会等待 `/mavros/state.connected: true` 和 3 秒稳定心跳才启动 controller；
后续样本在停止旧 PX4 后必须先观察到 `connected: false`，再按相同条件确认新 PX4 就绪。
连接生命周期由 batch 进程内的单一 ROS subscriber 观察，切换时要求收到新的状态序列，
不会通过反复启动 `rostopic echo` 子进程轮询。可通过 `timeouts.px4_ready` 调整稳定期。
UDP 模式下，batch 为每个 case 复制其 PX4
`etc` 到独立 rootfs，并仅将可识别的 `simulator_mavlink start -c ...` 改为
`simulator_mavlink start -u <transport.port>`；原 PX4 文件不改。无法识别的 PX4 启动结构会拒绝运行，
此时在 YAML 通过 `px4.startup_script` 提供该版本的 UDP 启动脚本。下一条样本在启动 PX4 前调用
`/gazebo/reset_world`。`--continue-on-error` 会把失败写入汇总后继续下一条；默认首个失败即停止。

### 可视化与三维飞行回放

`visualize` 读取已有 bag，不启动 ROS 节点，也不做动力学仿真；无人机的位置和姿态直接
来自 `/mavros/local_position/odom`。默认只绘制任务实际执行阶段，以保持与 `analyze` 的指标
时间范围一致；`--scope full` 可回放从起飞到降落的完整记录。所有 scope 都要求 bag 包含
`/flight_state`，以便使用同一套任务 phase 定义。

```bash
# 输出论文用的轨迹、跟踪、姿态/推力和指标概览图
python3 -m flight_eval visualize \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task plan_mission \
  --planner fast_planner_kino

# 同时生成 plots/flight_replay.gif；将整段记录压缩成 12 秒、15 FPS 的动画
python3 -m flight_eval visualize \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task plan_mission \
  --planner fast_planner_kino \
  --scope full --gif --gif-fps 15 --gif-duration 12
```

默认输出到 bag 同目录的 `plots/`，可用 `--output` 改到其他目录。非 `hover` 任务的静态图包括：
`trajectory_3d.png`、`trajectory_xy.png`、`position_tracking.png`、
`position_tracking_error.png`（存在参考轨迹时）、`attitude_thrust.png` 和
`yaw_tracking_error.png`（存在有效 yaw 参考时）、`metrics_overview.png`。`hover` 跳过前两张
轨迹图。若使用 `--gif`，还会生成 `flight_replay.gif`；动画中的四旋翼机架使用记录的
roll/pitch/yaw 绘制。

2D 和 3D 轨迹图都会绘制三类轨迹：橙色实线为实际飞行位置，蓝色虚线为从当前有效
`PlannerOutput` 重建的控制参考，紫色点划线为每条原始 `PlannerOutput` 消息中的轨迹段。
后者按消息窗口分别绘制，不会把相邻滚动 horizon 错连成一条轨迹。参考轨迹本身由
`PlannerOutput` 重建，因此两者在理想情况下可能重合；线型、颜色和图例仍会明确区分。

除 `hover` 外，`run`、`analyze` 与 `visualize` 会在输出目录生成任务执行 phase 的 2D/3D
轨迹图：
`trajectory_3d.png`、`trajectory_xy.png`、`position_tracking.png`、
`position_tracking_error.png`（存在参考时）、`attitude_thrust.png` 和
`yaw_tracking_error.png`（存在有效 yaw 参考时）。`hover` 不生成 `trajectory_3d.png` 或
`trajectory_xy.png`，但仍生成位置、姿态/推力、有效 yaw 误差和指标图；自动生成不会输出 GIF。

稳态/抖动和振荡频谱属于按需诊断图，需要单独调用：

```bash
# 悬停抖动和 Z 轴稳态误差
python3 -m flight_eval visualize ... --diagnostic hover_stability

# 与 oscillation_detection 相同预处理的频谱图
python3 -m flight_eval visualize ... --diagnostic oscillation
```

## 常见用法

### 1. 控制器悬停

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task hover
```

### 2. 控制器连续参考跟踪

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task analytic_circle
```

对 `analytic_circle / analytic_figure8 / analytic_spiral`，`flight_eval` 会在 `planner=none` 时自动注入 `analytic_reference`，无需手动指定 planner。

### 3. 离散 waypoint 轨迹生成

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory

python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_figure8 \
  --planner rpg_trajectory
```

这里测的是“离散 waypoint 集 -> 可执行轨迹输出”的能力，不是解析圆跟踪。`planner_only` 只是这类任务的默认报告视图。

### 4. 整链路 mission

```bash
python3 -m flight_eval run \
  --controller mav_linear_mpc \
  --task plan_mission \
  --planner fast_planner_kino \
  --max-collision-episodes 3
```

### 5. 仅分析已有 bag

```bash
python3 -m flight_eval analyze \
  --bag /path/to/flight_test.bag \
  --controller se3_hopf \
  --task plan_mission \
  --planner fast_planner_kino
```

`analyze` 会在 bag 所在目录生成或覆盖：

- `report.json`
- `agent_summary.md`

若 bag 同目录存在 `run_metadata.json`，会自动恢复运行时的任务时长和起飞高度；否则可用
`--duration` 显式指定；对 `discrete*` 和 `plan_mission`，它表示任务最大允许时长。

### 6. 修改判定逻辑后批量重分析

不必重跑仿真。rosbag 保留了离线重建任务结果所需的原始话题；对一个 batch 的所有 case，使用：

```bash
python3 -m flight_eval analyze \
  --batch-dir eval_runs/batches/my_batch \
  --recompute-outcome
```

`--batch-dir` 会递归查找每个 case 的 `run_metadata.json`，从中恢复 controller、task、planner、
起飞高度和任务时长，并将该 case 目录中的 `report.json`、`agent_summary.md` 和核心图直接覆盖。
它会将重算出的 `task_outcome` 回写到 `run_metadata.json`，并标记
`task_outcome_source=offline_recomputed` 和重算时间；该 case 的报告、摘要和核心图也会同步覆盖。

默认 `analyze` 同样优先显示这一运行时判定。只有显式传入 `--recompute-outcome` 时，才忽略
已存的 `task_outcome`，依据当前 evaluator 和 rosbag 重新推导任务结果。这个选项也可用于单个 bag：

```bash
python3 -m flight_eval analyze \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner rpg_trajectory \
  --recompute-outcome
```

## 当前任务

| task | 主要评价能力 | waypoint_type | 支持报告视图 |
|---|---|---|---|
| `hover` | 控制器悬停稳定性 | 无 | `controller_only` |
| `analytic_circle` | 控制器对连续解析圆参考的跟踪能力 | 无 | `controller_only` |
| `analytic_figure8` | 控制器对连续解析 8 字参考的跟踪能力 | 无 | `controller_only` |
| `analytic_spiral` | 控制器对连续解析螺旋参考的跟踪能力 | 无 | `controller_only` |
| `discrete_circle` | 离散 waypoint 集到可执行轨迹的生成/优化能力 | `circle` | `planner_only`, `integrated` |
| `discrete_figure8` | 离散 waypoint 集到可执行轨迹的生成/优化能力 | `eight` | `planner_only`, `integrated` |
| `plan_mission` | 障碍环境中的整链路任务执行能力 | `manual`（可显式覆盖） | `integrated` |

任务语义建议：

- `hover` 主要用于控制器悬停能力评估
- `analytic*` 主要用于控制器连续参考跟踪能力评估
- `discrete*` 主要用于轨迹生成 / 轨迹优化能力评估
- `plan_mission` 主要用于实时规划 + 实时跟踪 + 任务执行能力评估

`hover` 和 `analytic*` 是定时评价任务，`duration` 表示数据窗口。`discrete*` 和
`plan_mission` 是有限的有序航点任务，`duration` 表示完成任务的最长期限；成功后提前进入
降落。一次收到的 `nav_msgs/Path` 按数组顺序定义 mission，evaluator 只有在当前航点进入
容差并满足 dwell 后才切换到下一点；不能因为靠近终点、路径交叉或相邻点距离很小而跳过
中间航点。`discrete*` 保持连续路径进度语义，因为它们评估的是整条预设轨迹而不是在线航段。

SUPER 不直接消费整条 Path。`waypoint_generator/mission_manager` 是 SUPER 的接入适配器，
按顺序只派发当前 `PoseStamped` 航点；同一航段重发时保持 `header.seq`，切换航段时更换它。
SUPER 的该接入模式据此强制接受新航段，不依赖航点间的距离阈值。flight_eval 独立观察原始
Path 和 odom，不依赖 SUPER 专有状态。

任务名只接受表中的正式名称；不保留旧别名。

不同可运行配置使用不同算法名称，不使用 variant 覆盖：

```bash
--planner fast_planner_kino
--planner fast_planner_topo
--planner airfar_planner
--planner airfar_planner_mid360
--planner ego_planner_mid360
```

额外 roslaunch 参数可以用 `--planner-arg` 透传：

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory \
  --planner-arg use_nonlinear_opt:=true
```

## 统一接口与话题约定

当前统一规划输出接口是：

```text
/planner/output   (opendrone/PlannerOutput)
```

`flight_eval` 当前主链路不再依赖 `/command/trajectory`。

`planner_only` 中：

- 高亮 `/planner/output`、几何质量和规划发布质量
- 仍然保留控制和系统指标，但不把它们放在主视图中心

`integrated` 中：

- 高亮任务完成性、控制跟踪和系统执行结果
- `discrete*` 和 `plan_mission` 会报告独立的 `task_outcome`、完成时间和最终目标距离
- `discrete*` 同时保留规划输出计数和几何质量

## 运行流程

```text
1. 解析 controller / planner / task
2. 组装录包话题
3. 启动 rosbag
4. 启动 controller launch
5. 等控制器状态机进入任务阶段
6. 启动 planner launch，并等待第一条 `/planner/output`
7. 定时任务等待评价窗口结束；终点任务等待到达整体终点或超过期限
8. 到达终点或期限后自动降落（除非 --no-land）
9. 停止 rosbag / controller / planner 的整个进程组
10. 保存 `run_metadata.json`，自动分析并生成 report.json + agent_summary.md
```

Runner 自己维护一套与 controller `flight_state` 分离的生命周期状态机：

```text
PREPARING → RECORDING → WAITING_FOR_CONTROLLER → STARTING_PLANNER
  → WAITING_FOR_PLANNER → EXECUTING → REQUESTING_LAND → WAITING_FOR_LANDED
  → CAPTURING_ARTIFACTS → COMPLETED

任意运行阶段收到 EMERGENCY → ABORTING → CAPTURING_ARTIFACTS
超时 → FAILED；ROS shutdown / Ctrl-C → INTERRUPTED
```

`/flight_state` 只作为该状态机的输入：任务状态到达后才能启动 planner，`LANDED` 才能结束
降落等待，`EMERGENCY` 可从任意阶段中止运行。运行元数据会保存完整的状态迁移历史，便于定位
卡在 controller、planner output 或降落阶段的原因。

Runner 将结果拆成两个互不推导的状态：

```yaml
run_status:
  status: completed        # completed | failed | interrupted
  reason: artifacts captured

task_outcome:
  status: succeeded        # succeeded | not_succeeded | unknown | not_applicable
  reason: goal_reached
  completion_time: 23.6    # 仅成功时存在
  evidence: {}
```

`run_status=completed` 只表示启动、录包、清理和结果保存流程正常走完。定时型任务没有天然
终点，`task_outcome=not_applicable`；`discrete*` 和 `plan_mission` 把 `duration` 作为期限，
在期限内完成整条 Path 为 `succeeded`，超时为 `not_succeeded`，缺少 Path/odom 等证据时为 `unknown`。
EMERGENCY 不会被伪装成 Runner 故障：制品仍可正常保存时 `run_status=completed`，有限终点
任务的 `task_outcome=not_succeeded`。

controller 必须按统一状态码发布：`0=WAITING_FOR_CONNECTED`、`1=WAITING_FOR_OFFBOARD`、
`2=TAKEOFF`、`3=MISSION_EXECUTION`、`4=LANDING`、`5=LANDED`、`6=EMERGENCY`。

所有由 runner 启动的 `rosbag` 和 `roslaunch` 都在独立进程组中。batch 收到首个
`SIGINT` 或 `SIGTERM` 后会取消整个批次，统一清理当前 case 的全部进程组并停止 PX4；
清理期间的重复终止信号不会打断 `SIGINT → SIGTERM → SIGKILL` 升级。即使 `roslaunch`
父进程已经退出，仍会按已登记的进程组清理其剩余子节点。
`run_metadata.json.process_cleanup` 会记录已登记和仍残留的进程组；如果 `SIGKILL` 后仍有
进程组存在，`run_status` 为 `failed / process_cleanup_incomplete`，不会伪报运行完成。
`environment.launch` 启动的 Gazebo 和 MAVROS 是 batch 外部环境，并且配置为 respawn，
所以 task timeout 和 case 切换都不会关闭它们；batch 每个 case 只重启自己拥有的 PX4。

## 分析逻辑

### phase 切分

`BagAnalyzer` 先找状态机进入分析阶段的时刻，再按任务自己的 `execution_start_policy` 决定真正执行起点：

- `hover`：直接以 mission 状态起点为准
- `analytic*`：以第一条参考/规划输出为起点
- `discrete*`：优先使用 trigger / waypoint / planner output 这些任务信号
- `plan_mission`：优先使用 trigger / waypoint / planner output 这些任务信号

如果一直没有任务信号，则判定 `execution_started = false`。若发生 `EMERGENCY`，阶段会在触发时截断。

`/flight_state` 是 phase 切分的必需话题；缺失时会拒绝分析和可视化。

### 指标组织

每个任务计算固定的完整指标集。报告只按 `group` 整理，不筛选指标，也不汇总得分。

`discrete*` 常见指标包括：

- `planner_output_rate`：执行 phase 内按 bag 消息时间计算的 `/planner/output` 发布频率
- `planner_output_message_count`：执行 phase 内的 `/planner/output` 消息数
- `planner_output_point_count`：执行 phase 内收到的 `/planner/output` 消息所携带的有效轨迹点数
- 生成质量：`generated_trajectory_duration`、连续折线定义的
  `waypoint_distance_mean/p95/max`、`speed_max`、`acceleration_max`、`jerk_rms`
  和 position/velocity/acceleration 最大连续性跳变
- 执行跟踪和诊断：位置/速度跟踪、显式 yaw 跟踪、姿态/推力波动和跟踪误差频谱峰值
- 可选任务几何描述，例如 circle 的半径误差/角度覆盖率、figure8 的闭合误差/中心交叉距离

生成质量只使用 `/planner/output`。解析器完整保留 `trajectory_id`、`time_from_start`、
`valid_mask` 及各阶导数，按 `trajectory_id + absolute_desired_time` 去除 rolling horizon
重叠采样；不会从控制参考或实际飞行轨迹回退补齐生成轨迹。Waypoint 距离是 waypoint
到生成轨迹连续折线的最短欧氏距离，不使用硬编码“命中”容差。

有限终点任务 `discrete*` 和 `plan_mission` 会额外包含：

- `task_outcome`（独立于指标）
- `completion_time`（成功时）
- `final_goal_distance`
- 跟踪误差和姿态/推力稳定性指标
- `execution_started` 与 `emergency_status`

`analytic*` 和 `hover` 主要关注：

- 位置或轨迹跟踪误差
- 姿态波动
- 推力波动
- 振荡检测

`trajectory_tracking` 的主指标为 `position_tracking_rmse_3d`、
`position_tracking_p95_3d` 和 `velocity_tracking_rmse_3d`；只有 PlannerOutput 显式设置
`VALID_YAW` 时才输出 `yaw_tracking_rmse`，不会从轨迹切线推测 yaw。诊断指标为
`reference_coverage_ratio`、`max_reference_gap`、`roll_pitch_fluctuation_rms`、
`thrust_std` 和 `tracking_error_spectrum_peak`。参考缺口不会被跨段插值掩盖，也不再使用
固定 20 秒“稳态”延迟。

时间跟踪和空间形状是两个独立维度。上述 tracking 指标按相同时刻配对，会对滞后和
相位差敏感；`spatial_shape_rmse_3d` 和 `spatial_shape_p95_3d` 在每个连续参考段内
忽略时间配对，计算实际/参考连续折线的双向最短距离。双向度量可避免只飞轨迹的
局部也获得过好结果。因此，tracking 误差大而 spatial shape 误差小，表示形状基本正确但
时序滞后；两者都大才表示时序跟踪和空间路径都差。该空间指标不判断遍历顺序，
需要时可由具体 Task 另加顺序敏感的几何描述器。

报告不把异构指标汇总成总分，也不内置单项阈值评级。`run_status` 只描述运行流程，
`task_outcome` 只描述任务是否完成；两者都不是算法排名。碰撞次数来自
`/flight_eval/contact_pulse`，其数据源和 episode 合并参数会随指标一并记录。

## 录制话题

基础通用录包话题：

```text
/mavros/local_position/odom
/mavros/state
```

框架按 `kind` 自动录制 controller 的 `/flight_state` 或 planner 的 `/planner/output`，
再叠加 Task 自己的输入 topic、manifest 的 `record_topics` 和命令行 `--extra-topics`
（runner 会去重）：

- controller benchmark launch 必须发布 `/flight_state` (`std_msgs/Int8`)
- planner benchmark launch 必须发布 `/planner/output` (`opendrone/PlannerOutput`)
- 路径类 Task 使用 `/waypoint_generator/waypoints` (`nav_msgs/Path`) 作为原始任务定义

Path 的 `header.frame_id` 必须与 `/mavros/local_position/odom.header.frame_id` 使用同一局部
ENU 坐标系。当前 SITL 两者均为 `map`；部分 planner 内部使用 `world`，由现有零偏移静态 TF
完成接入，但通用 evaluator 不猜测或隐式变换 frame。两者显式不一致时任务结果为
`unknown / frame_mismatch`。
- `--extra-topics`

`tasks.py` 定义任务语义、阶段、任务协议话题和固定指标组合；指标的具体数据来源会写入报告，
不再由另一层“报告视图”筛选。

## 接入新的算法

外部算法不修改 `controllers.py`、`planners.py` 或评测核心，而是提供一个 YAML manifest。
统一 schema 位于 [`schemas/algorithm_manifest.schema.yaml`](schemas/algorithm_manifest.schema.yaml)，
完整示例位于
[`examples/external_algorithm_manifest.yaml`](examples/external_algorithm_manifest.yaml)。

manifest 放在外部算法包内，和 benchmark launch 一起版本管理。真正必填的只有 YAML
中的算法名称、`kind`、`launch.package` 和 `launch.file`。可选字段只有：

- `args`：默认 launch 参数
- `tasks`：允许参与的 Task 白名单；省略表示不限制
- `record_topics`：算法自己的诊断 topic

```yaml
version: 1
algorithms:
  my_planner_depth:
    kind: planner
    launch:
      package: my_planner
      file: flight_eval.launch
    args:
      sensor: depth_camera
    tasks:
      - plan_mission
    record_topics:
      - /my_planner/diagnostics
```

统一接口、消息类型、坐标系和公共状态输入由本接入文档规定，不在每个 manifest 重复声明。
框架会使用本次实际传入的 launch 参数展开 controller/planner launch（包括 include 和条件分支），
保存其中节点的私有参数命名空间以及 launch 直接声明的其他参数；不会归档完整 ROS 参数树 `/`。
所用 manifest 随结果归档。论文、许可证、来源和实现偏差属于项目文档或论文附录，不进入运行
manifest。

单次运行使用：

```bash
python3 -m flight_eval run \
  --manifest /path/to/my_package/flight_eval.yaml \
  --controller my_controller \
  --planner my_planner_depth --task plan_mission
```

批次 YAML 在根对象的 `manifests` 列表中加载；相对路径以批次 YAML 所在目录为基准。
`analyze`、`visualize`、`list-controllers` 和 `list-planners` 也支持 `--manifest`。

`PlannerOutput` 的每个点必须满足下面的时间不变量：

```text
absolute_desired_time = trajectory_start_time + point.time_from_start
```

公共消息只包含 `header`、`trajectory_id`、`is_horizon`、`trajectory_start_time` 和
`points`。`is_horizon=true` 表示新消息替代旧滚动窗口；否则多点表示同一轨迹的时间序列，
单点在下一条消息到来前保持有效。原始 B-spline、多项式、算法 id、算法分类和 debug 数据
属于算法诊断 topic，不进入公共控制协议。原算法输出格式不同时，在 benchmark launch 中
加入 adapter，把它采样为 `PlannerOutputPoint[]`。

`header.frame_id` 是所有空间向量的 ROS ENU 坐标系（通常为 `map` 或 `world`）；
同一条消息的 point 不允许混用坐标系，`trajectory_id` 在参考轨迹改变时递增。
每个 point 用 `valid_mask` 声明有效的 position、velocity、acceleration、jerk、snap、yaw、
yaw rate 或 angular velocity，未声明的字段必须被消费者忽略。位置是必需字段，其余均可选。

运行时调参使用 `--controller-arg key:=value` 或 `--planner-arg key:=value`；两者都会
写入 `run_metadata.json` 和参数快照清单。`takeoff_height` 属于 Task 条件，只能通过
`--takeoff-height` 设置，不能被 `--controller-arg` 覆盖。

## 输出文件

默认输出目录：

```text
eval_runs/{controller}_{task}_{planner}_{YYYYmmdd_HHMMSS}/
```

通常会包含：

```text
flight_test*.bag
report.json
agent_summary.md
run_metadata.json
rosparams.json
parameter_snapshot.json
algorithm_manifests/   # 使用外部算法时，归档本次实际加载的 manifest
```

执行 `visualize` 后，运行目录还会包含：

```text
plots/trajectory_3d.png
plots/trajectory_xy.png
plots/position_tracking.png
plots/position_tracking_error.png
plots/attitude_thrust.png
plots/yaw_tracking_error.png
plots/metrics_overview.png
plots/flight_replay.gif  # 仅 --gif 时生成
```

## 常用命令

```bash
# 1) hover
python3 -m flight_eval run \
  --controller pid_controller \
  --task hover \
  --takeoff-height 1.5 \
  --duration 15

# 2) discrete circle + RPG trajectory
python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_circle \
  --planner rpg_trajectory

# 3) discrete figure8 + Fast-Planner topo
python3 -m flight_eval run \
  --controller mav_linear_mpc \
  --task discrete_figure8 \
  --planner fast_planner_topo

# 4) plan_mission
python3 -m flight_eval run \
  --controller mav_nonlinear_mpc \
  --task plan_mission \
  --planner super_planner_od \
  --no-land

# 5) analyze
python3 -m flight_eval analyze \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task plan_mission

```

## 备注

- `flight_eval` 当前已经默认站在“全面转 `/planner/output`”的协议上，不再围绕 `/command/trajectory` 做兼容设计。
- 如果你要新增 controller-only 的解析参考任务，推荐复用 `analytic_reference_publisher.py`，而不是让 planner 重建解析轨迹。
- `plan_mission` 由任务 profile 默认注入 `use_preset_waypoints:=true`、`auto_trigger_waypoints:=true`
  和有限任务所需的 `waypoint_type:=manual`；`--planner-arg` 可显式覆盖同名值。
- 当前预设航点和解析参考都默认以无人机当前位置为起点。
