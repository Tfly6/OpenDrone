# 结果数据与复算

`samples/tracking_state.csv`、`samples/tracking_reference.csv` 是各含十个采样点的合成示例，用于快速验证跟踪误差计算：

```bash
python3 verify_example.py
```

论文随附数据包包含逐试次结果、测量信号和配置快照。下表路径相对于解压后的 `data/` 目录：

| 材料 | 作用 |
| --- | --- |
| `revision_audit/trials.csv` | 每次实验的状态、跟踪指标、纳入决定和原始归档映射 |
| `revision_audit/plan_to_result.csv` | 核对计划中的试次是否有实际配置、bag、报告 |
| `revision_audit/joint_outcomes.csv` | 原生成功、几何完成与接触的联合计数 |
| `tracking_sensitivity.csv`、`coverage_sensitivity.csv` | 覆盖率门槛变化时保留哪些试次、指标如何变化 |
| `online_combinations.csv`、`planner_summary.csv` | 完整在线组合与规划器汇总 |
| `revision_review_20261116/` | 固定任务跨场景对照中的里程计/真值及窗口复核 |
| `worked_example/` | 一个实际飞行案例的原始状态、参考点、接触及复算结果 |
| `configuration_records.zip` | 646 次尝试的完整配置、参数、报告和运行元数据快照 |
| `replay_source_snapshot.zip` | 生成归档审计结果的实现快照 |

复算论文数值时，将 `OPENDRONE_DATA_DIR` 指向数据包的 `data/` 目录：

```bash
export OPENDRONE_DATA_DIR=/path/to/extracted_package/data
python3 summarize_planners.py
python3 verify_release.py
python3 build_evidence.py
```

前两项统计复算使用 Python 标准库；图表生成所需依赖见 `requirements.txt`。数据包中的完整说明给出信号、时钟、任务窗口、纳入条件及各列定义。

从原始 rosbag 重新计算时，归档按 `OPENDRONE_WORKSPACE/eval_runs/batches/<batch>/cases/<case>/` 放置。运行方法见 [README.md](README.md) 的“分析与复算”。
