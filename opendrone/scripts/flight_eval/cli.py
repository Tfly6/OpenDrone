#!/usr/bin/env python3
"""flight_eval command-line entry point."""

import argparse
import sys
import os
import json
from datetime import datetime

from .controllers import CONTROLLER_REGISTRY, get_controller_launch
from .manifests import ManifestError, load_manifest_files
from .planners import PLANNER_REGISTRY, get_planner_launch
from .tasks import TASK_REGISTRY, create_task
from .runner import FlightRunner
from .batch import (
    BatchConfigError,
    FlightEvalBatchRunner,
    format_batch_plan,
    load_batch_definition,
)
from .environment import prepare_environment
from .analyzer import BagAnalyzer
from .visualizer import BagVisualizer


_WS_EVAL_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '..', 'eval_runs'))
_OpenDrone_ROOT = os.path.abspath(os.path.join(os.path.abspath(__file__), '..', '..', '..', '..'))
_PARAMETER_SNAPSHOT_MANIFEST_FILENAME = 'parameter_snapshot.json'


def _write_parameter_snapshot_manifest(
    dest_dir: str,
    result: dict,
    controller_info: dict,
    planner_info: dict,
) -> str:
    manifest = {
        'format_version': 3,
        'task': result['task'],
        'metric_profile': result['metric_profile'],
        'controller': {
            'name': result['controller'],
            'launch_pkg': controller_info.get('launch_pkg', ''),
            'launch_file': controller_info.get('launch_file', ''),
            'launch_args': result.get('controller_launch_args', {}),
            'requested_args': result.get('requested_controller_args', {}),
        },
        'planner': {
            'name': result['planner'],
            'launch_pkg': planner_info.get('launch_pkg', '') if planner_info else '',
            'launch_file': planner_info.get('launch_file', '') if planner_info else '',
            'launch_args': result.get('planner_launch_args', {}),
            'requested_args': result.get('requested_planner_args', {}),
        },
        'topic_contract': result.get('topic_contract', {}),
        'run_status': result.get('run_status', {}),
        'task_outcome': result.get('task_outcome', {}),
        'rosparam_snapshot': result.get('parameter_snapshot', {}),
        'algorithm_manifests': result.get('algorithm_manifests', []),
    }

    manifest_path = os.path.join(dest_dir, _PARAMETER_SNAPSHOT_MANIFEST_FILENAME)
    with open(manifest_path, 'w', encoding='utf-8') as manifest_file:
        json.dump(manifest, manifest_file, indent=2, ensure_ascii=False)
    return manifest_path


def _parse_roslaunch_args(arg_items):
    """解析 --planner-arg key:=value / key=value 参数。"""
    parsed = {}
    for item in arg_items or []:
        if ':=' in item:
            key, value = item.split(':=', 1)
        elif '=' in item:
            key, value = item.split('=', 1)
        else:
            raise ValueError(f"planner 参数格式错误: {item}, 应为 key:=value")
        if not key:
            raise ValueError(f"planner 参数名为空: {item}")
        parsed[key] = value
    return parsed


def _generate_core_metric_plots(
    analyzer: BagAnalyzer,
    report,
    task_name: str,
    output_dir: str,
    controller_name: str,
    planner_name: str,
    hover_height: float,
    duration: float,
) -> dict:
    """为每个 task 自动输出核心轨迹、误差和姿态图，复用当前分析缓存。"""
    full_data, phase_data = analyzer.get_cached_data()
    visualizer = BagVisualizer(
        bag_file=analyzer.bag_file,
        controller_name=controller_name,
        task_name=task_name,
        planner_name=planner_name,
        hover_height=hover_height,
        duration=duration,
    )
    try:
        return visualizer.render_core_plots(
            output_dir=output_dir,
            report=report,
            full_data=full_data,
            phase_data=phase_data,
            scope='execution',
        )
    except (RuntimeError, ValueError) as exc:
        return {'warning': f'未生成核心图: {exc}'}


def _postprocess_completed_run(result: dict, record_dir: str) -> dict:
    """复用单次 run 的归档、分析和核心作图流程，供 batch 调用。"""
    controller_name = result['controller']
    takeoff_height = result['takeoff_height']
    task_duration = result['task_duration']
    controller_manifest_info = get_controller_launch(controller_name)
    planner_manifest_info = {}
    if result['planner'] != 'none':
        planner_manifest_info = get_planner_launch(result['planner'])
    manifest_path = _write_parameter_snapshot_manifest(
        record_dir,
        result,
        controller_manifest_info,
        planner_manifest_info,
    )
    analyzer = BagAnalyzer(
        bag_file=result['bag_file'],
        controller_name=controller_name,
        task_name=result['task'],
        planner_name=result['planner'],
        hover_height=takeoff_height,
        duration=task_duration,
    )
    report = analyzer.analyze()
    report_path = analyzer.save_report(report, record_dir)
    analyzer.print_report(report)
    core_artifacts = _generate_core_metric_plots(
        analyzer=analyzer,
        report=report,
        task_name=result['task'],
        output_dir=record_dir,
        controller_name=controller_name,
        planner_name=result['planner'],
        hover_height=takeoff_height,
        duration=task_duration,
    )
    return {
        'report_file': report_path,
        'parameter_snapshot_manifest': manifest_path,
        'agent_summary': os.path.join(record_dir, 'agent_summary.md'),
        'core_artifacts': core_artifacts,
    }


def _load_run_metadata(bag_file: str) -> dict:
    """Load the metadata colocated with a recorded bag, if available."""
    path = os.path.join(os.path.dirname(os.path.abspath(bag_file)), 'run_metadata.json')
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as metadata_file:
            value = json.load(metadata_file)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[warn] 无法读取运行元数据 {path}: {exc}")
        return {}
    return value if isinstance(value, dict) else {}


def _analyze_recording(
    bag_file: str,
    controller_name: str,
    task_name: str,
    planner_name: str,
    output_dir: str,
    hover_height=None,
    duration=None,
    recompute_outcome: bool = False,
) -> str:
    """Analyze one archived recording and overwrite its report artifacts."""
    analyzer = BagAnalyzer(
        bag_file=bag_file,
        controller_name=controller_name,
        task_name=task_name,
        planner_name=planner_name,
        hover_height=hover_height,
        duration=duration,
        recompute_outcome=recompute_outcome,
    )
    report = analyzer.analyze()
    report_path = analyzer.save_report(report, output_dir)
    if recompute_outcome:
        metadata_path = os.path.join(
            os.path.dirname(os.path.abspath(bag_file)), 'run_metadata.json'
        )
        if os.path.isfile(metadata_path):
            metadata = _load_run_metadata(bag_file)
            metadata['task_outcome'] = report.task_outcome
            metadata['task_outcome_source'] = 'offline_recomputed'
            metadata['task_outcome_recomputed_at'] = datetime.now().isoformat()
            with open(metadata_path, 'w', encoding='utf-8') as metadata_file:
                json.dump(metadata, metadata_file, indent=2, ensure_ascii=False)
    analyzer.print_report(report)
    core_artifacts = _generate_core_metric_plots(
        analyzer=analyzer,
        report=report,
        task_name=task_name,
        output_dir=os.path.dirname(report_path),
        controller_name=controller_name,
        planner_name=planner_name,
        hover_height=hover_height,
        duration=duration,
    )
    print(f"\n报告已保存: {report_path}")
    print(f"Agent 摘要已保存: {os.path.join(os.path.dirname(report_path), 'agent_summary.md')}")
    for name, path in core_artifacts.items():
        prefix = '[warn]' if name == 'warning' else '自动生成'
        print(f"{prefix} {name}: {path}")
    return report_path


def _batch_recordings(batch_dir: str):
    """Yield archived cases beneath a batch root in stable order."""
    for root, _dirs, files in os.walk(os.path.abspath(batch_dir)):
        if 'run_metadata.json' not in files:
            continue
        metadata_path = os.path.join(root, 'run_metadata.json')
        try:
            with open(metadata_path, 'r', encoding='utf-8') as metadata_file:
                metadata = json.load(metadata_file)
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f'无法读取 {metadata_path}: {exc}') from exc
        if not isinstance(metadata, dict):
            raise ValueError(f'{metadata_path} 必须是 JSON object')
        bag_file = metadata.get('bag_file') or os.path.join(root, 'flight_test.bag')
        yield root, os.path.abspath(bag_file), metadata

def main():
    parser = argparse.ArgumentParser(
        description='无人机控制器飞行评估框架',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 运行 se3_hopf 控制器起飞悬停评估
  python -m flight_eval run --controller se3_hopf

  # 指定起飞高度和任务执行时间
  python -m flight_eval run --controller pid_controller --takeoff-height 1.5 --duration 15

  # 仅分析已有 bag 文件
  python -m flight_eval analyze --bag flight_test_2026-01-01-12-00-00.bag --controller se3_hopf

  # 生成论文图并导出三维飞行回放 GIF
  python -m flight_eval visualize --bag flight_test.bag --controller se3_hopf --gif

  # 列出支持的控制器
  python -m flight_eval list-controllers
        """)
    subparsers = parser.add_subparsers(dest='command', help='子命令')

    run_parser = subparsers.add_parser('run', help='启动控制器并录制评估数据')
    run_parser.add_argument('--controller', '-c', required=True,
                            help='控制器名称')
    run_parser.add_argument(
        '--manifest', action='append', default=[],
        help='加载外部算法 manifest YAML；可重复指定',
    )
    run_parser.add_argument('--task', '-t', default='hover',
                            choices=list(TASK_REGISTRY.keys()),
                            help='评估任务名称, 默认 hover')
    run_parser.add_argument('--planner', '-p', default='none',
                            help='规划器名称, 默认 none')
    run_parser.add_argument('--planner-arg', action='append', default=[],
                            help='传给规划器 launch 的参数, 格式 key:=value, 可重复')
    run_parser.add_argument('--controller-arg', action='append', default=[],
                            help='传给控制器 launch 的参数，格式 key:=value，可重复；不能覆盖任务起飞高度')
    run_parser.add_argument('--takeoff-height', type=float, default=None,
                            help='起飞高度 (m), 默认按任务配置取 2.0')
    run_parser.add_argument('--duration', type=float, default=None,
                            help='定时任务评价窗口或终点任务期限 (s)，默认按 task 配置')
    run_parser.add_argument('--record-dir', type=str, default=None,
                            help='bag 录制保存目录, 默认 eval_runs/{控制器}_{task}_{日期}/')
    run_parser.add_argument('--run-name', type=str, default='',
                            help='运行名称 (用作子目录名), 默认为 控制器_task_日期')
    run_parser.add_argument('--no-land', action='store_true',
                            help='评估完成后不自动降落')
    run_parser.add_argument('--extra-topics', nargs='*', default=[],
                            help='额外录制的 ROS 话题')
    run_parser.add_argument(
        '--max-collision-episodes',
        type=int,
        default=3,
        help='累计多少次独立碰撞后终止任务，默认 3',
    )
    run_parser.add_argument(
        '--collision-episode-gap',
        type=float,
        default=0.5,
        help='同一障碍物重新计为一次碰撞所需的无消息间隔 (s)，默认 0.5',
    )

    batch_parser = subparsers.add_parser(
        'batch',
        help='按 YAML 配置批量运行 flight_eval；Gazebo/MAVROS 与 PX4 分离管理',
    )
    batch_parser.add_argument('--config', '-f', required=True,
                              help='批次实验 YAML 配置文件')
    batch_parser.add_argument('--output-dir', default=None,
                              help='覆盖 YAML 的 output_dir；目标必须不存在或为空')
    batch_parser.add_argument('--dry-run', action='store_true',
                              help='只校验并展示 YAML 展开的样本，不启动 ROS/PX4/Gazebo')
    batch_parser.add_argument('--continue-on-error', action='store_true',
                              help='单条样本失败后继续后续样本；默认在首个失败处停止')
    batch_parser.add_argument('--no-analyze', action='store_true',
                              help='仅运行和录包，不自动生成报告与图')

    prepare_environment_parser = subparsers.add_parser(
        'prepare-environment',
        help='从 YAML 指定的 PX4 生成 UDP Gazebo 覆盖模型；随后由使用者手动启动环境',
    )
    prepare_environment_parser.add_argument('--config', '-f', required=True,
                                            help='批次实验 YAML 配置文件')

    analyze_parser = subparsers.add_parser('analyze', help='分析已录制的 bag 文件或整批 case')
    analyze_input = analyze_parser.add_mutually_exclusive_group(required=True)
    analyze_input.add_argument('--bag', '-b', help='单个 bag 文件路径')
    analyze_input.add_argument(
        '--batch-dir',
        help='批次输出目录；递归查找 case 的 run_metadata.json 并逐个重分析',
    )
    analyze_parser.add_argument('--controller', '-c',
                                help='控制器名称；单 bag 必填，batch 从各 case 元数据恢复')
    analyze_parser.add_argument(
        '--manifest', action='append', default=[],
        help='加载外部算法 manifest YAML；可重复指定',
    )
    analyze_parser.add_argument('--task', '-t', default=None,
                                choices=list(TASK_REGISTRY.keys()),
                                help='评估任务名称；单 bag 默认 hover，batch 从各 case 元数据恢复')
    analyze_parser.add_argument('--planner', '-p', default=None,
                                help='产生 planner 接口的规划器；单 bag 默认 none，batch 从各 case 元数据恢复')
    analyze_parser.add_argument('--output', '-o', type=str, default=None,
                                help='输出 JSON 报告路径, 默认保存为 bag 同目录下的 report.json')
    analyze_parser.add_argument('--hover-height', type=float, default=None,
                                help='悬停目标高度 (m), 默认从 bag 中自动检测')
    analyze_parser.add_argument('--duration', type=float, default=None,
                                help='任务评价窗口或期限；默认从 run_metadata.json 恢复')
    analyze_parser.add_argument(
        '--recompute-outcome', action='store_true',
        help='忽略元数据中的运行时 task_outcome，用当前 evaluator 从 bag 重新推导',
    )
    visualize_parser = subparsers.add_parser('visualize', help='生成 bag 轨迹、指标图和可选三维回放 GIF')
    visualize_parser.add_argument('--bag', '-b', required=True, help='bag 文件路径')
    visualize_parser.add_argument('--controller', '-c', required=True, help='控制器名称')
    visualize_parser.add_argument(
        '--manifest', action='append', default=[],
        help='加载外部算法 manifest YAML；可重复指定',
    )
    visualize_parser.add_argument('--task', '-t', default='hover',
                                  choices=list(TASK_REGISTRY.keys()), help='评估任务名称，默认 hover')
    visualize_parser.add_argument('--planner', '-p', default='none',
                                  help='产生规划接口的规划器')
    visualize_parser.add_argument('--output', '-o', default=None,
                                  help='图片输出目录，默认 bag 同目录')
    visualize_parser.add_argument('--duration', type=float, default=None,
                                  help='任务评价窗口或期限；默认从 run_metadata.json 恢复')
    visualize_parser.add_argument('--hover-height', type=float, default=None,
                                  help='悬停目标高度；默认从 run_metadata.json 恢复')
    visualize_parser.add_argument('--scope', choices=('execution', 'full'), default='execution',
                                  help='绘制任务执行阶段或完整飞行过程，默认 execution')
    visualize_parser.add_argument('--gif', action='store_true', help='额外生成 flight_replay.gif')
    visualize_parser.add_argument('--gif-fps', type=int, default=15, help='GIF 帧率，默认 15')
    visualize_parser.add_argument('--gif-duration', type=float, default=12.0,
                                  help='GIF 播放时长（秒）；原始时间会重采样到该时长，默认 12')
    visualize_parser.add_argument('--dpi', type=int, default=220, help='PNG/GIF 输出 DPI，默认 220')
    visualize_parser.add_argument('--diagnostic', action='append', default=[],
                                  choices=('hover_stability', 'oscillation', 'all'),
                                  help='额外指标诊断图；可重复指定。hover_stability 仅适用于 hover')
    list_controllers_parser = subparsers.add_parser(
        'list-controllers', help='列出支持的控制器'
    )
    list_controllers_parser.add_argument('--manifest', action='append', default=[])
    subparsers.add_parser('list-tasks', help='列出支持的评估任务')
    list_planners_parser = subparsers.add_parser(
        'list-planners', help='列出支持的规划器'
    )
    list_planners_parser.add_argument('--manifest', action='append', default=[])

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    try:
        load_manifest_files(
            getattr(args, 'manifest', []),
            CONTROLLER_REGISTRY,
            PLANNER_REGISTRY,
        )
    except (ManifestError, RuntimeError) as exc:
        parser.error(f'算法 manifest 无效: {exc}')

    if (
        hasattr(args, 'controller')
        and args.controller is not None
        and args.controller not in CONTROLLER_REGISTRY
    ):
        parser.error(
            f'未知控制器: {args.controller}. 可用: {sorted(CONTROLLER_REGISTRY)}'
        )
    if (
        hasattr(args, 'planner')
        and args.planner is not None
        and args.planner not in PLANNER_REGISTRY
    ):
        parser.error(f'未知规划器: {args.planner}. 可用: {sorted(PLANNER_REGISTRY)}')

    print(f"OpenDrone 根目录: {_OpenDrone_ROOT}")
    print(f"评估结果将保存在: {_WS_EVAL_ROOT}")

    if args.command == 'list-controllers':
        print("支持的控制器:")
        print("-" * 60)
        for name, info in CONTROLLER_REGISTRY.items():
            print(f"  {name:20s} - {info['description']}")
            print(f"  {'':20s}   launch: {info['launch_file']}")
            print()
        return

    if args.command == 'list-tasks':
        print("支持的评估任务:")
        print("-" * 60)
        for name, task_cls in TASK_REGISTRY.items():
            task = task_cls()
            print(f"  {name:20s} - {task.description}")
            print(f"  {'':20s}   metric_profile: {task.metric_profile}")
            print(f"  {'':20s}   default_duration: {task.default_duration:.1f}s")
        return

    if args.command == 'list-planners':
        print("支持的规划器:")
        print("-" * 60)
        for name, info in PLANNER_REGISTRY.items():
            print(f"  {name:20s} - {info['description']}")
            if info.get('launch_file'):
                print(f"  {'':20s}   launch: {info['launch_file']}")
            if info.get('tasks'):
                print(f"  {'':20s}   tasks: {', '.join(info['tasks'])}")
            print()
        return

    if args.command == 'batch':
        try:
            definition = load_batch_definition(args.config, args.output_dir)
        except (BatchConfigError, ValueError) as exc:
            parser.error(f'批次配置无效: {exc}')
        if args.no_analyze:
            definition.analysis_enabled = False
        print(format_batch_plan(definition))
        if args.dry_run:
            print('\n[dry-run] 配置校验和样本展开完成；未启动 ROS/PX4/Gazebo。')
            return

        def on_completed_run(result, record_dir):
            print(f"\n[batch] 正在分析: {record_dir}")
            return _postprocess_completed_run(result, record_dir)

        batch_runner = FlightEvalBatchRunner(
            definition=definition,
            continue_on_error=args.continue_on_error,
            result_handler=on_completed_run,
        )
        try:
            summary = batch_runner.run()
        except KeyboardInterrupt:
            print('\n[batch] 收到 Ctrl-C，已停止当前批次拥有的 PX4 进程。')
            return
        failed = [case for case in summary['cases'] if case['status'] == 'failed']
        print(f"\n批次汇总: {summary['summary_file']}")
        print(
            f"运行流程完成: {len(summary['cases']) - len(failed)}，"
            f"运行失败: {len(failed)}"
        )
        if failed:
            for case in failed:
                print(f"失败样本: {case['case_name']}: {case.get('error', '未知原因')}")
        return

    if args.command == 'prepare-environment':
        try:
            definition = load_batch_definition(args.config)
            prepared = prepare_environment(definition)
        except (BatchConfigError, ValueError) as exc:
            parser.error(f'环境准备失败: {exc}')
        print(f"UDP 环境目录: {prepared['environment_dir']}")
        print(f"请先执行: source {prepared['shell_file']}")
        print('然后在单独终端启动 Gazebo + MAVROS:')
        print(prepared['launch_command'])
        return

    if args.command == 'run':
        if args.max_collision_episodes < 1:
            parser.error('--max-collision-episodes 必须是正整数')
        if args.collision_episode_gap <= 0.0:
            parser.error('--collision-episode-gap 必须大于 0')
        planner_args = _parse_roslaunch_args(args.planner_arg)
        controller_args = _parse_roslaunch_args(args.controller_arg)
        planner_info = get_planner_launch(args.planner, planner_args)
        takeoff_height = args.takeoff_height if args.takeoff_height is not None else 2.0
        task = create_task(
            args.task,
            takeoff_height=takeoff_height,
        )
        task_duration = (
            args.duration if args.duration is not None
            else task.default_duration
        )

        run_name = args.run_name
        if not run_name:
            date_str = datetime.now().strftime('%Y%m%d_%H%M%S')
            run_name = f"{args.controller}_{args.task}_{args.planner}_{date_str}"

        record_dir = args.record_dir or os.path.join(_WS_EVAL_ROOT, run_name)

        runner = FlightRunner(
            controller_name=args.controller,
            task_name=args.task,
            planner_name=args.planner,
            planner_args=planner_args,
            controller_args=controller_args,
            takeoff_height=takeoff_height,
            duration=task_duration,
            record_dir=record_dir,
            auto_land=not args.no_land,
            extra_topics=args.extra_topics,
            max_collision_episodes=args.max_collision_episodes,
            collision_episode_gap=args.collision_episode_gap,
        )
        try:
            result = runner.run()
        except KeyboardInterrupt:
            print('\n[run] 收到 Ctrl-C，已停止当前运行拥有的子进程。')
            return

        if result:
            print("\n" + "=" * 60)
            print("运行流程完成（不代表任务成功）")
            print("=" * 60)
            print(f"Bag 文件: {result['bag_file']}")
            print(f"控制器:   {result['controller']}")
            print(f"指标集合: {result['metric_profile']}")
            print(f"任务:     {result['task']}")
            print(f"规划器:   {result['planner']}")
            print(f"飞行时长: {result['flight_duration']:.1f}s")
            print(f"EMERGENCY: {'是' if result['emergency_occurred'] else '否'}")
            print(
                f"运行状态:   {result['run_status']['status']} "
                f"({result['run_status']['reason']})"
            )
            print(
                f"任务结果:   {result['task_outcome']['status']} "
                f"({result['task_outcome']['reason']})"
            )
            print()
            print("正在分析数据...")
            postprocess = _postprocess_completed_run(result, runner.record_dir)
            print(f"\n报告已保存: {postprocess['report_file']}")
            print(f"参数快照清单已保存: {postprocess['parameter_snapshot_manifest']}")
            print(f"Agent 摘要已保存: {postprocess['agent_summary']}")
            for name, path in postprocess['core_artifacts'].items():
                prefix = '[warn]' if name == 'warning' else '自动生成'
                print(f"{prefix} {name}: {path}")

    elif args.command == 'analyze':
        if args.bag:
            if not args.controller:
                parser.error('analyze --bag 必须指定 --controller')
            task_name = args.task or 'hover'
            planner_name = args.planner or 'none'
            run_metadata = _load_run_metadata(args.bag)
            analysis_duration = (
                args.duration if args.duration is not None
                else run_metadata.get('task_duration', create_task(task_name).default_duration)
            )
            analysis_height = (
                args.hover_height if args.hover_height is not None
                else run_metadata.get('takeoff_height')
            )
            _analyze_recording(
                bag_file=args.bag,
                controller_name=args.controller,
                task_name=task_name,
                planner_name=planner_name,
                output_dir=args.output or os.path.dirname(os.path.abspath(args.bag)),
                hover_height=analysis_height,
                duration=analysis_duration,
                recompute_outcome=args.recompute_outcome,
            )
        else:
            if not os.path.isdir(args.batch_dir):
                parser.error(f'analyze --batch-dir 不是目录: {args.batch_dir}')
            if args.output:
                parser.error('analyze --batch-dir 不支持 --output；报告始终覆盖每个 case 目录')
            if any(value is not None for value in (
                args.controller, args.task, args.planner, args.hover_height, args.duration,
            )):
                parser.error(
                    'analyze --batch-dir 从每个 case 的 run_metadata.json 恢复参数；'
                    '不能同时指定 controller/task/planner/height/duration'
                )
            succeeded = 0
            failed = []
            recordings = sorted(_batch_recordings(args.batch_dir))
            if not recordings:
                parser.error(
                    f'analyze --batch-dir 下未找到 run_metadata.json: {args.batch_dir}'
                )
            for case_dir, bag_file, metadata in recordings:
                try:
                    if not os.path.isfile(bag_file):
                        raise ValueError(f'bag 不存在: {bag_file}')
                    controller_name = metadata.get('controller')
                    task_name = metadata.get('task')
                    planner_name = metadata.get('planner')
                    if not all(isinstance(value, str) and value for value in (
                        controller_name, task_name, planner_name,
                    )):
                        raise ValueError('元数据缺少 controller、task 或 planner')
                    print(f"\n{'=' * 60}\n重分析: {case_dir}")
                    _analyze_recording(
                        bag_file=bag_file,
                        controller_name=controller_name,
                        task_name=task_name,
                        planner_name=planner_name,
                        output_dir=case_dir,
                        hover_height=metadata.get('takeoff_height'),
                        duration=metadata.get('task_duration'),
                        recompute_outcome=args.recompute_outcome,
                    )
                    succeeded += 1
                except (ImportError, OSError, RuntimeError, ValueError) as exc:
                    failed.append((case_dir, str(exc)))
                    print(f"[error] 重分析失败: {exc}", file=sys.stderr)
            print(f"\n批量重分析完成: 成功 {succeeded}，失败 {len(failed)}")
            for case_dir, reason in failed:
                print(f"  - {case_dir}: {reason}")
            if failed:
                return 1

    elif args.command == 'visualize':
        run_metadata = {}
        metadata_path = os.path.join(
            os.path.dirname(os.path.abspath(args.bag)), 'run_metadata.json'
        )
        if os.path.isfile(metadata_path):
            try:
                with open(metadata_path, 'r', encoding='utf-8') as metadata_file:
                    run_metadata = json.load(metadata_file)
            except (OSError, json.JSONDecodeError) as exc:
                print(f"[warn] 无法读取运行元数据: {exc}")
        visualizer = BagVisualizer(
            bag_file=args.bag,
            controller_name=args.controller,
            task_name=args.task,
            planner_name=args.planner,
            hover_height=(args.hover_height if args.hover_height is not None
                          else run_metadata.get('takeoff_height')),
            duration=(args.duration if args.duration is not None
                      else run_metadata.get('task_duration', create_task(args.task).default_duration)),
        )
        output_dir = args.output or os.path.dirname(os.path.abspath(args.bag))
        artifacts = visualizer.render(
            output_dir=output_dir,
            scope=args.scope,
            gif=args.gif,
            gif_fps=args.gif_fps,
            gif_duration=args.gif_duration,
            dpi=args.dpi,
            diagnostics=args.diagnostic,
        )
        print('\n可视化产物:')
        for name, path in artifacts.items():
            print(f"  {name}: {path}")

if __name__ == '__main__':
    main()
