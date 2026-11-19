# OpenDrone 论文实验代码

本目录包含论文实验的运行、分析和绘图代码。完整实验结果另存于论文数据包；此处仅保留小型合成示例，用于快速验证分析流程。

## 实验设计

| 分组 | 设计与评价目的 | 尝试次数 |
| --- | --- | ---: |
| `hover` | 七类控制器跟踪固定目标，评价稳态偏差和位置波动 | 21 |
| `tracking` | 七类控制器跟踪圆、八字、螺旋解析参考，评价动态跟踪误差 | 105 |
| `generation` | MAV/RPG 在圆、八字任务中生成参考并由控制器执行，评价轨迹性质与下游跟踪 | 140 |
| `online` | 在三种环境、两种传感器配置下运行规划任务，评价原生状态、几何履约与接触 | 300 |
| `controlled` | 固定单点/双点任务，在圆柱和森林环境中对照，检查任务匹配及状态来源对评价的影响 | 80 |
| `all` | 上述全部飞行 | **646** |

运行清单包含 646 次飞行尝试。解析参考跟踪中两次、轨迹生成与执行中三次因有效样本不足被排除，论文共纳入 641 次。重跑清单保留全部尝试；18 个合成规则案例和适配器检查另行运行。

`online` 比较完整场景包：圆柱与森林的任务几何随场景配置变化。`controlled` 在两个环境中使用相同任务定义、深度传感器和时间预算，包含 Hopf/PID、EGO/Fast-kino，每种组合重复五次。

`experiments/full_suite.json` 指定任务、控制器、规划器、重复编号、launch 参数及航点参数。运行器逐试次加载任务参数，执行当前 checkout 的算法实现。数据包中的参数快照与实现标识用于核对归档结果的测量条件。

## 代码位置

| 文件/目录 | 用途 |
| --- | --- |
| `run_suite.py` / `run_suite.sh` | 全套实验的清单、计划与执行入口 |
| `experiments/full_suite.json` | 各实验组的算法组合、重复次数和运行配置 |
| `experiments/task_*.yaml` | 固定任务对照的单点、双点任务定义 |
| `run_revision.py` / `run_revision.sh` | 评价协议检查、归档审计和固定任务执行入口 |
| `revision_support/` | 参考重构、任务与接触审计、状态来源复核及运行支持 |
| `record_ground_truth.py` | Iris 仿真真值采样 |
| `summarize_planners.py`、`verify_release.py` | 读取外部结果包，复算论文统计和完整案例 |
| `build_evidence.py`、`draw_framework.py` | 表格与图件生成 |
| `export_worked_example.py` | 从外部原始 bag 导出完整案例 |
| `samples/`、`verify_example.py` | 两个十行 CSV 的合成示例，验证基本跟踪误差计算 |
| `checks/` | 任务、适配器、运行器及可移植性检查 |

通用执行与评价实现位于 `opendrone/scripts/flight_eval/`，控制器和规划器分别位于项目的 `controller/`、`planner/`。

## 查看实验与验证小示例（标准库即可）

在本目录运行：

```bash
python3 run_suite.py list
python3 run_suite.py list online --sensor Mid360
python3 verify_example.py
python3 -m unittest checks.test_portability
```

## 计划与运行全部实验

要求 ROS Noetic、当前 checkout 对应的 catkin 编译产物、PX4 SITL 和 Gazebo Classic。入口按当前仓库定位代码，向上寻找工作区；需要时显式设置路径：

```bash
export OPENDRONE_WORKSPACE=/path/to/catkin_ws
export PX4_SOURCE_DIR=/path/to/PX4-Autopilot

# 解析全部配置和 launch 参数，不启动仿真。
bash run_suite.sh plan all

# 按实验设计选择分组；all 执行全部实验。
bash run_suite.sh run hover
bash run_suite.sh run tracking
bash run_suite.sh run generation
bash run_suite.sh run online
bash run_suite.sh run controlled
# bash run_suite.sh run all
```

`--sensor`、`--world` 可缩小范围；`--limit 1` 表示每个选中批次先运行一条试次。`--px4-source` 可覆盖 PX4 路径。

入口管理 Gazebo/MAVROS、真值采样和逐试次 PX4，启动前关闭已有 Gazebo/MAVROS。实验记录写入工作区 `eval_runs/batches/`，计划、日志和派生结果默认写入 `paper_artifacts/`，可用 `OPENDRONE_ARTIFACTS` 改写。

只运行代码检查和规则案例：

```bash
bash run_revision.sh check
```

## 分析与复算

从 bag 重新审计或导出案例需要原始归档和 ROS 环境：

```bash
bash run_revision.sh audit --bags
bash run_revision.sh review
# source ROS/catkin 后：
python3 export_worked_example.py
```

从论文全量结果包复算统计和图表：

```bash
export OPENDRONE_DATA_DIR=/path/to/extracted_data_package/data
python3 summarize_planners.py
python3 verify_release.py
python3 build_evidence.py
```

CSV 复算只依赖 Python 标准库；绘图及 ROS 入口所需 Python 依赖见 `requirements.txt`。图表默认写入 `outputs/`，可用 `--output` 指定目录。更多结果文件说明见 [DATA_README.md](DATA_README.md)。
