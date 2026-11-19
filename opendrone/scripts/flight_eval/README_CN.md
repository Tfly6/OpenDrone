[English](README.md) | [简体中文](README_CN.md)

# OpenDrone 飞行评测

`flight_eval` 用于运行和分析可复现的 PX4 SITL 实验，覆盖 OpenDrone 控制器、轨迹生成器和在线规划器。它会记录各项任务所需的 ROS 证据并生成结构化结果，但不会给算法计算总分或排名。

## 主要功能

- 支持控制器、轨迹生成和在线任务评测。
- 支持单次运行和 YAML 实验矩阵。
- 自动录制 rosbag 和运行元数据。
- 无需重跑仿真即可离线重新分析。
- 生成轨迹、跟踪、姿态、推力和指标图。
- 可选生成三维飞行回放 GIF。
- 通过 YAML manifest 接入仓库外部算法。

## 环境要求

请先完成 OpenDrone 编译并加载 ROS 工作空间。Python 分析依赖记录在 `requirements.txt`：

```bash
python3 -m pip install -r requirements.txt
```

将当前目录的上级路径加入 `PYTHONPATH`：

```bash
cd ~/catkin_ws
source devel/setup.bash
export PYTHONPATH=$PWD/src/OpenDrone/opendrone/scripts:$PYTHONPATH
cd src/OpenDrone/opendrone/scripts
```

## 查询可用组件

当前代码中的注册表是可用组件的唯一准确信息来源：

```bash
python3 -m flight_eval list-controllers
python3 -m flight_eval list-planners
python3 -m flight_eval list-tasks
```

通过内置帮助查看完整且与当前版本一致的参数列表：

```bash
python3 -m flight_eval --help
python3 -m flight_eval run --help
python3 -m flight_eval batch --help
python3 -m flight_eval analyze --help
python3 -m flight_eval visualize --help
```

## 快速开始

运行实验前，需要先启动兼容的 Gazebo、PX4 SITL 和 MAVROS 环境。

控制器悬停：

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task hover \
  --duration 20
```

解析轨迹跟踪：

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task analytic_circle
```

根据离散航点生成轨迹：

```bash
python3 -m flight_eval run \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory
```

在线任务：

```bash
python3 -m flight_eval run \
  --controller mav_linear_mpc \
  --task sequential_goal_mission \
  --planner fast_planner_kino
```

## 分析与可视化

分析已有 bag：

```bash
python3 -m flight_eval analyze \
  --bag /path/to/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory
```

生成图表和可选飞行回放：

```bash
python3 -m flight_eval visualize \
  --bag /path/to/flight_test.bag \
  --controller se3_hopf \
  --task discrete_circle \
  --planner mav_trajectory \
  --scope full --gif
```

## 批量实验

先在不启动 ROS 进程的情况下验证实验矩阵：

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml \
  --dry-run
```

准备好匹配的外部仿真环境后执行：

```bash
python3 -m flight_eval batch \
  --config ./flight_eval/examples/batch_example.yaml
```

示例配置用于展示完整能力，内容较多。正式实验应复制该文件，并只保留需要的实验组合与仿真配置。

## 输出文件

单次运行默认保存在 `eval_runs/` 下，通常包含：

```text
flight_test*.bag
run_metadata.json
report.json
agent_summary.md
rosparams.json
parameter_snapshot.json
plots/
```

批量运行还会保存展开后的计划、批次摘要、各 case 目录、PX4 日志和相互隔离的 PX4 rootfs。

`run_status` 表示运行与制品保存流程是否完成；`task_outcome` 独立表示有限任务是否达成语义目标。两者都不是算法综合评级。

## 详细文档

- [用户指南](docs/user-guide_CN.md)：任务选择、单次运行、批量环境、分析、可视化和输出文件。
- [开发者指南](docs/developer-guide_CN.md)：架构、话题契约、任务语义、指标、生命周期和外部算法 manifest。
- [批量实验示例](examples/batch_example.yaml)：完整的实验矩阵配置。
- [外部算法示例](examples/external_algorithm_manifest.yaml)：注册内置表之外的控制器或规划器。
- [Manifest Schema](schemas/algorithm_manifest.schema.yaml)：可由工具读取的 manifest 契约。

## 设计原则

- Task 定义实验语义及其固定指标集合。
- `/planner/output` 是统一轨迹接口。
- 报告只提供测量结果和数据来源，不计算综合分数或排名。
- 保留原始 bag 与元数据，使修改后的分析逻辑能够离线重算。
- 运行流程状态与任务是否成功始终分开记录。

## 安全说明

该框架会启动和停止 ROS 进程，并可能执行解锁、Offboard、起飞和降落命令。除非已经独立审查 launch 文件及目标硬件的安全行为，否则仅应在隔离的 SITL 环境中使用。
