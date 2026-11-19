[English](developer-guide.md) | [简体中文](developer-guide_CN.md)

# 飞行评测开发者指南

本文定义 `flight_eval` 的内部模型和接入契约。实验操作说明见[用户指南](user-guide_CN.md)。

## 概念模型

```text
FlightRunner / BagAnalyzer
  |-- task: 实验语义与指标 profile
  |-- controller: 控制器 launch 与诊断话题
  |-- planner: 参考或规划 launch 与诊断话题
  |-- run_status: 执行及制品保存结果
  `-- task_outcome: 任务语义完成结果
```

Task 负责定义实验含义，包括阶段、开始策略、评价窗口或期限、协议话题、结果判定器和完整指标集合。Controller 与 planner 条目负责描述算法的启动方式和需要录制的算法专属话题。

不能根据进程是否成功推断任务是否完成。即使飞行器未完成任务，运行流程仍可能正常保存全部制品。

## 源码职责

| 文件 | 职责 |
|---|---|
| `cli.py` | 命令、参数解析、后处理和制品入口 |
| `runner.py` | 进程编排、飞行生命周期、录包和清理 |
| `batch.py` | YAML 验证、矩阵展开、环境 session 和逐 case PX4 |
| `environment.py` | 生成外部 Gazebo/MAVROS 环境 |
| `tasks.py` | Task、阶段、结果判定器和指标组合 |
| `controllers.py` | 内置控制器注册表 |
| `planners.py` | 内置规划器注册表 |
| `analyzer.py` | Bag 解析、阶段提取和报告生成 |
| `metrics.py` | 指标实现 |
| `outcomes.py` | 任务协议和语义结果判定 |
| `visualizer.py` | 静态图与三维回放 |
| `manifests.py` | 外部算法 manifest 加载和验证 |

## Task 与指标 Profile

| 指标 Profile | Task | 主要关注点 |
|---|---|---|
| `hover_regulation` | `hover` | 位置稳定性、姿态/推力波动和振荡 |
| `trajectory_tracking` | `analytic_circle`、`analytic_figure8`、`analytic_spiral` | 时间对齐的参考跟踪和空间形状 |
| `trajectory_generation` | `discrete_circle`、`discrete_figure8` | 规划输出质量、航点几何、连续性和执行结果 |
| `online_mission` | `sequential_goal_mission`、`reference_path_mission` | 任务完成、用时、碰撞证据和跟踪 |

每个 Task 都报告固定的完整指标集合。输入缺失时，指标应带数据来源标记为不可用，而不是静默切换数据源。框架不会将异构指标合成为综合分数。

## 话题契约

通用录制话题至少包括：

```text
/mavros/local_position/odom
/mavros/state
```

所选 controller、planner、Task、外部 manifest 和 `--extra-topics` 会扩展该集合，重复话题会被去除。

### 控制器生命周期

兼容评测的控制器通过 `/flight_state` 发布 `std_msgs/Int8`：

| 编码 | 状态 |
|---:|---|
| 0 | `WAITING_FOR_CONNECTED` |
| 1 | `WAITING_FOR_OFFBOARD` |
| 2 | `TAKEOFF` |
| 3 | `MISSION_EXECUTION` |
| 4 | `LANDING` |
| 5 | `LANDED` |
| 6 | `EMERGENCY` |

阶段提取必须使用 `/flight_state`。它是 runner 生命周期的输入，但不等同于 runner 自身状态。

### 统一规划输出

规划器直接或通过 adapter 向 `/planner/output` 发布 `opendrone/PlannerOutput`。每个轨迹点满足：

```text
absolute_desired_time = trajectory_start_time + point.time_from_start
```

接口规则：

- `header.frame_id` 为消息内全部空间字段指定同一个 ROS 局部 ENU 坐标系。
- 位置字段必填，其余字段可选。
- `valid_mask` 声明每个有效的导数、偏航、偏航角速度或角速度字段。
- 消费者必须忽略未设置对应 bit 的字段。
- 新原生轨迹生效时 `trajectory_id` 必须变化，同一轨迹的不同窗口保持不变。
- `is_horizon=true` 表示新消息替换旧滚动窗口。
- 非 horizon 的多点消息表示时间序列；单点持续有效直到被替换。
- `trajectory_status` 只表示局部轨迹生命周期。

原算法 ID、地图、求解器诊断以及未采样的多项式或 B-spline 数据应保留在算法专属诊断话题中。

### Mission 协议

支持完整任务的规划器通过 `/planner/mission_state` 发布 `opendrone/MissionState`。Mission 类型、已完成项数、总项数和成功状态必须保持单调且相互一致。只有与活动 Task 匹配的成功状态才能结束评测。

原始任务路径使用 `/waypoint_generator/waypoints`（`nav_msgs/Path`），其坐标系必须与 MAVROS 局部里程计兼容。Evaluator 只将几何数据作为结果证据，不推进目标，也不自行生成任务进度。

### 碰撞证据

碰撞事件来自 `/flight_eval/contact_pulse`。同一障碍物的连续接触按 `collision_episode_gap` 合并为一次 episode，仿真插件会排除地面接触。达到 `max_collision_episodes` 后安全结束任务。

## Runner 生命周期

```text
PREPARING -> RECORDING -> WAITING_FOR_CONTROLLER -> STARTING_PLANNER
  -> WAITING_FOR_PLANNER -> EXECUTING -> REQUESTING_LAND
  -> WAITING_FOR_LANDED -> CAPTURING_ARTIFACTS -> COMPLETED
```

紧急状态进入 `ABORTING`，随后保存制品；超时进入 `FAILED`；ROS shutdown 或用户中断进入 `INTERRUPTED`。

Runner 在独立进程组中启动 rosbag、controller 和 planner。清理按 `SIGINT`、`SIGTERM`、`SIGKILL` 逐级升级。如果最后仍有进程组残留，运行结果必须是 `process_cleanup_incomplete`，不能伪报完成。

批量模式下，外部 Gazebo/MAVROS 环境不归单个 case 所有。每个 case 只拥有自己的 PX4 进程和隔离 rootfs。批次终止时会清理当前 case；批次结束后默认尝试关闭外部环境，使用 `--keep-gazebo` 时除外。

## 阶段提取与分析

Analyzer 首先根据 `/flight_state` 定位任务阶段，然后应用 Task 的执行开始策略：

- `hover`：以 mission 状态阶段起点为准。
- `analytic_*`：以第一条有效参考或规划输出为准。
- `discrete_*`：以第一条 trigger、waypoint 或规划输出为准。
- mission Task：以第一条 trigger、waypoint 或规划输出为准。

如果所需任务信号始终没有出现，则 `execution_started=false`。发生紧急状态时，分析阶段在对应时间戳截断。缺少 `/flight_state` 的 bag 无法建立可比较的执行区间，因此分析和可视化会拒绝处理。

参考样本由 `/planner/output` 重建，并通过轨迹标识与绝对期望时间对滚动 horizon 去重。不同分段之间的参考缺口会被保留，不会跨段插值掩盖。

时间对齐的跟踪指标与空间路径形状是两个独立维度。跟踪指标会惩罚滞后和相位差；空间形状指标不按时间配对连续折线，因此描述的是几何形状，而不是遍历时序或顺序。

## 结果语义

```yaml
run_status:
  status: completed
  reason: artifacts captured

task_outcome:
  status: succeeded
  reason: goal_reached
  completion_time: 23.6
  evidence: {}
```

`run_status` 可以是 `completed`、`failed` 或 `interrupted`；`task_outcome` 可以是 `succeeded`、`not_succeeded`、`unknown` 或 `not_applicable`。定时任务通常没有语义终点，因此使用 `not_applicable`。

## 外部算法 Manifest

外部算法应通过 YAML 注册，不应修改内置注册表。Schema 位于 [`algorithm_manifest.schema.yaml`](../schemas/algorithm_manifest.schema.yaml)，完整示例位于 [`external_algorithm_manifest.yaml`](../examples/external_algorithm_manifest.yaml)。

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
      - sequential_goal_mission
    record_topics:
      - /my_planner/diagnostics
```

算法必填字段为名称、`kind`、`launch.package` 和 `launch.file`。可选字段用于提供默认 launch 参数、Task 白名单和诊断录制话题。

单次运行加载 manifest：

```bash
python3 -m flight_eval run \
  --manifest /path/to/my_package/flight_eval.yaml \
  --controller my_controller \
  --planner my_planner_depth \
  --task sequential_goal_mission
```

批量 YAML 接受根级 `manifests` 列表；查询、分析和可视化命令也接受 `--manifest`。实际加载的 manifest 和解析后的 launch 参数会随运行归档。

## 增加内置能力

- 只有引入不同实验语义时才新增 Task；在 `tasks.py` 实现并注册到 `TASK_REGISTRY`。
- 在 `metrics.py` 实现指标，并由所属 Task 组合，不应创建可切换的报告视图。
- 内置算法启动入口应加入对应 controller 或 planner 注册表。
- 评测包装 launch 按类别保存在 `opendrone/launch/benchmarks/{controller_benchmark,trajectory_generation_benchmark,online_planner_benchmark}`。
- 算法专属格式转换放在 launch 层 adapter 中，不应向公共消息加入求解器专属字段。
- Manifest 格式变化时同步更新 schema 和示例。

修改后应依次执行 dry-run 配置检查、已有 bag 重分析和最小代表性 SITL 任务，再运行完整实验矩阵。
