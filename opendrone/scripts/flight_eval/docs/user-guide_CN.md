[English](user-guide.md) | [简体中文](user-guide_CN.md)

# 飞行评测用户指南

本文说明如何选择和运行实验。接口与实现约束见[开发者指南](developer-guide_CN.md)。

## 运行前准备

1. 编译 OpenDrone 并加载 catkin 工作空间。
2. 将 `opendrone/scripts` 加入 `PYTHONPATH`。
3. 启动与实验匹配的仿真环境，确保 world、机体传感器、传输端口和 MAVROS URL 一致。
4. 确认 MAVROS 已连接，ROS 时间正常推进。
5. 使用查询命令确认当前代码实际支持的名称。

```bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH
cd src/OpenDrone/opendrone/scripts

python3 -m flight_eval list-controllers
python3 -m flight_eval list-planners
python3 -m flight_eval list-tasks
```

## 选择任务

| Task | 主要用途 | Planner |
|---|---|---|
| `hover` | 悬停控制 | `none` |
| `analytic_circle` | 连续圆形参考跟踪 | 自动使用 `analytic_reference` |
| `analytic_figure8` | 连续 8 字参考跟踪 | 自动使用 `analytic_reference` |
| `analytic_spiral` | 连续螺旋参考跟踪 | 自动使用 `analytic_reference` |
| `discrete_circle` | 根据圆形离散航点生成轨迹 | 必填 |
| `discrete_figure8` | 根据 8 字离散航点生成轨迹 | 必填 |
| `sequential_goal_mission` | 依次执行一组目标点 | 必填 |
| `reference_path_mission` | 执行连续参考路径 | 必填 |

对于定时任务，`--duration` 表示评价窗口；对于有限轨迹和任务，它表示完成期限。

## 单次运行

通用命令格式：

```bash
python3 -m flight_eval run \
  --controller CONTROLLER \
  --task TASK \
  --planner PLANNER
```

常用参数：

| 参数 | 作用 |
|---|---|
| `--takeoff-height METERS` | 覆盖任务默认起飞高度 |
| `--duration SECONDS` | 设置评价窗口或完成期限 |
| `--record-dir PATH` | 指定输出目录 |
| `--run-name NAME` | 指定运行子目录名称 |
| `--controller-arg key:=value` | 向控制器传递 launch 参数 |
| `--planner-arg key:=value` | 向规划器传递 launch 参数 |
| `--extra-topics TOPIC ...` | 增加 rosbag 话题 |
| `--no-land` | 不请求自动降落 |
| `--keep-gazebo` | 完成后保留外部 Gazebo 环境 |

示例：

```bash
python3 -m flight_eval run \
  --controller pid_controller \
  --task hover \
  --takeoff-height 1.5 \
  --duration 15

python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_figure8 \
  --planner rpg_trajectory

python3 -m flight_eval run \
  --controller mav_nonlinear_mpc \
  --task sequential_goal_mission \
  --planner super_planner_od \
  --max-collision-episodes 3
```

完整参数以 `python3 -m flight_eval run --help` 为准。

## 批量实验

批量模式将控制器、规划器和重复次数展开为 case 矩阵。Gazebo 和 MAVROS 位于 batch 外部并在 case 之间复用；batch 会为每个 case 启动独立的 PX4 SITL 进程和 rootfs。

### 准备 UDP 环境

YAML 必须指定 PX4 源码目录，并使用整个批次内保持不变的 Gazebo、MAVROS 和 transport 配置。首先生成环境文件：

```bash
python3 -m flight_eval prepare-environment \
  --config ./flight_eval/examples/batch_example.yaml
```

命令会打印生成路径。在负责外部环境的终端中加载 `environment.env` 并运行 `launch_command.sh`：

```bash
source /tmp/flight_eval_udp_environment/environment.env
bash /tmp/flight_eval_udp_environment/launch_command.sh
```

该 launch 包含 Gazebo、机体模型和 MAVROS，但不包含 PX4。生成的 Iris 覆盖模型使用配置的 UDP 端口，不会修改 PX4 源码或 build 目录。

### 验证并运行矩阵

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml \
  --dry-run

python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml
```

Dry-run 会验证名称、任务兼容性、配置和 case 展开结果，但不会启动 ROS 进程。默认情况下任一 case 失败会终止批次；如需继续后续 case，使用 `--continue-on-error`。如需在批次结束后保留外部环境，使用 `--keep-gazebo`。

### 最小矩阵结构

```yaml
version: 1
name: controller_comparison
defaults:
  simulation:
    gazebo:
      world: empty
      sensor: none
      respawn_gazebo: true
    mavros:
      fcu_url: udp://:14540@localhost:14557
      respawn_mavros: true
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
experiments:
  - name: hover
    task: hover
    controllers: [se3_hopf, pid_controller]
    planners: [none]
    repeat: 3
```

case 数量由 `controllers × planners × repeat` 决定。单项也可以使用包含 `name` 和 `args` 的对象。全部仿真与运行字段见 [`batch_example.yaml`](../examples/batch_example.yaml)。

## 分析录制数据

分析单个 bag：

```bash
python3 -m flight_eval analyze \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory
```

如果 bag 同目录存在 `run_metadata.json`，分析会恢复原任务时长、起飞高度、运行时结果和算法上下文；否则需要通过命令行补充缺失值。

重新分析整个批次，并使用当前 evaluator 重算任务结果：

```bash
python3 -m flight_eval analyze \
  --batch-dir eval_runs/batches/my_batch \
  --recompute-outcome
```

重算会覆盖各 case 的报告、摘要和核心图，但不会修改原始 bag。

## 可视化录制数据

```bash
python3 -m flight_eval visualize \
  --bag eval_runs/example/flight_test.bag \
  --controller se3_hopf \
  --task sequential_goal_mission \
  --planner fast_planner_kino
```

添加 `--scope full` 可绘制从起飞到降落的完整过程，而不只绘制任务执行阶段。添加 `--gif` 会生成 `plots/flight_replay.gif`；通过 `--gif-fps` 和 `--gif-duration` 控制动画时间参数。

核心制品可能包括：

```text
plots/trajectory_3d.png
plots/trajectory_xy.png
plots/position_tracking.png
plots/position_tracking_error.png
plots/attitude_thrust.png
plots/yaw_tracking_error.png
plots/metrics_overview.png
plots/flight_replay.gif
```

缺少所需参考字段的图不会生成；悬停任务不生成路径形状图。

## 结果目录

单次运行默认目录：

```text
eval_runs/{controller}_{task}_{planner}_{timestamp}/
```

批量运行默认目录：

```text
eval_runs/batches/{batch_name}_{timestamp}/
|-- batch_plan.json
|-- batch_summary.json
|-- cases/
|   `-- {case}/
|       `-- px4_rootfs/
`-- logs/
```

每个已完成 case 通常保存 bag、`run_metadata.json`、`report.json`、`agent_summary.md`、参数快照和图表。外部算法 manifest 也会随 case 归档。

## 常见问题

- **没有控制器或规划器输出：** 确认所选算法支持当前任务，并检查 launch 输出。
- **MAVROS 一直未就绪：** 检查 PX4、Gazebo 和 MAVROS 的 FCU URL、传输方式及端口是否一致。
- **ROS 时间停止：** 确认 Gazebo 未暂停，且 `/clock` 正常推进。
- **没有分析阶段：** bag 必须包含 `/flight_state`，并且状态机必须到达任务阶段。
- **没有参考轨迹指标：** 确认已经录制 `/planner/output`，且轨迹点设置了所需的 `valid_mask`。
- **批次无法切换 world 或传感器：** 需要不同外部 Gazebo 环境的实验应拆分到不同 batch 文件。
