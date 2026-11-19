#!/usr/bin/env python3
"""Contracts for planner-owned mission state."""

import os
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = str(Path(__file__).resolve().parents[2] / 'scripts')
sys.path.insert(0, SCRIPT_DIR)

from flight_eval.outcomes import (  # noqa: E402
    MISSION_STATUS_ACTIVE,
    MISSION_STATUS_SUCCEEDED,
    MISSION_TYPE_REFERENCE_PATH,
    MISSION_TYPE_SEQUENTIAL_GOAL,
    MissionStateEvaluator,
    TaskOutcomeStatus,
)
from flight_eval.planners import PLANNER_REGISTRY, get_planner_launch  # noqa: E402
from flight_eval.tasks import TASK_REGISTRY  # noqa: E402


class MissionStateContractTest(unittest.TestCase):
    def test_position_never_advances_sequential_mission(self):
        evaluator = MissionStateEvaluator(MISSION_TYPE_SEQUENTIAL_GOAL)
        evaluator.update_path([(1, 0, 2), (2, 0, 2)], 0.0, 'map')
        evaluator.start(1.0)
        evaluator.update_position((2, 0, 2), 2.0, 'map', speed=0.0)
        self.assertEqual(evaluator.outcome.status, TaskOutcomeStatus.UNKNOWN)
        self.assertEqual(evaluator.outcome.evidence['completed_items'], 0)

    def test_sequential_progress_is_planner_owned_and_monotonic(self):
        evaluator = MissionStateEvaluator(MISSION_TYPE_SEQUENTIAL_GOAL)
        evaluator.update_path([(1, 0, 2), (2, 0, 2)], 0.0, 'map')
        evaluator.start(1.0)
        evaluator.notify_mission_state(
            2.0, MISSION_TYPE_SEQUENTIAL_GOAL,
            MISSION_STATUS_ACTIVE, 1, 2,
        )
        self.assertEqual(evaluator.outcome.evidence['completed_items'], 1)
        evaluator.notify_mission_state(
            3.0, MISSION_TYPE_SEQUENTIAL_GOAL,
            MISSION_STATUS_SUCCEEDED, 2, 2,
        )
        self.assertEqual(evaluator.outcome.status, TaskOutcomeStatus.SUCCEEDED)

    def test_type_and_total_are_part_of_the_protocol(self):
        evaluator = MissionStateEvaluator(MISSION_TYPE_SEQUENTIAL_GOAL)
        evaluator.update_path([(1, 0, 2)], 0.0, 'map')
        evaluator.start(1.0)
        evaluator.notify_mission_state(
            1.1, MISSION_TYPE_SEQUENTIAL_GOAL,
            MISSION_STATUS_SUCCEEDED, 1, 2,
        )
        evaluator.notify_mission_state(
            1.2, MISSION_TYPE_REFERENCE_PATH,
            MISSION_STATUS_SUCCEEDED, 1, 1,
        )
        self.assertEqual(evaluator.outcome.status, TaskOutcomeStatus.UNKNOWN)
        self.assertEqual(evaluator.outcome.evidence['rejected_planner_state_updates'], 2)

    def test_reference_path_is_one_semantic_item(self):
        evaluator = MissionStateEvaluator(MISSION_TYPE_REFERENCE_PATH)
        evaluator.update_path([(1, 0, 2), (2, 0, 2)], 0.0, 'map')
        evaluator.start(1.0)
        evaluator.notify_mission_state(
            2.0, MISSION_TYPE_REFERENCE_PATH,
            MISSION_STATUS_SUCCEEDED, 1, 1,
        )
        self.assertEqual(evaluator.outcome.status, TaskOutcomeStatus.SUCCEEDED)

    def test_task_split_and_planner_support_are_explicit(self):
        self.assertNotIn('plan_mission', TASK_REGISTRY)
        topo_tasks = PLANNER_REGISTRY['fast_planner_topo']['tasks']
        kino_tasks = PLANNER_REGISTRY['fast_planner_kino']['tasks']
        self.assertIn('reference_path_mission', topo_tasks)
        self.assertNotIn('sequential_goal_mission', topo_tasks)
        self.assertIn('sequential_goal_mission', kino_tasks)
        self.assertNotIn('reference_path_mission', kino_tasks)
        self.assertEqual(
            PLANNER_REGISTRY['fast_planner_kino']['evaluation_topics']['mission_state'],
            '/planner/mission_state',
        )

    def test_instrumentation_preserves_native_waypoint_switch_predicates(self):
        repository = os.path.dirname(os.path.dirname(SCRIPT_DIR))
        sources = {
            'ego': os.path.join(
                repository,
                'planner/ego_planner_swarm/src/plan_manage/ego_replan_fsm.cpp',
            ),
            'ego_v2': os.path.join(
                repository,
                'planner/ego_planner_v2/src/plan_manage/ego_replan_fsm.cpp',
            ),
            'airfar': os.path.join(
                repository,
                'planner/airfar_planner/src/airfar_planner.cpp',
            ),
        }
        contents = {}
        for name, path in sources.items():
            with open(path, encoding='utf-8') as source_file:
                contents[name] = source_file.read()

        self.assertIn(
            '(end_pt_ - pos).norm() < no_replan_thresh_', contents['ego'])
        self.assertIn(
            '(final_goal_ - pos).norm() < no_replan_thresh_',
            contents['ego_v2'],
        )

        reach_callback = contents['airfar'].split(
            'void DPMaster::ReachGoalStatusCallBack', 1)[1].split(
                '/* allocate static utility PointCloud', 1)[0]
        self.assertNotIn('is_robot_stop_', reach_callback)

    def test_planner_native_tolerance_is_not_an_evaluator_threshold(self):
        launch = get_planner_launch(
            'airfar_planner', {'goal_tolerance': '0.65'})
        self.assertEqual(launch['args']['goal_tolerance'], '0.65')
        self.assertEqual(
            launch['runtime_parameter_expectations'][
                '/airfar_path_follower/goalTolerance'],
            0.65,
        )

    def test_super_queue_preserves_native_mission_switch_semantics(self):
        repository = os.path.dirname(os.path.dirname(SCRIPT_DIR))
        workspace = os.path.dirname(os.path.dirname(repository))
        core_path = os.path.join(
            repository,
            'planner/super_planner/super_planner_od/src/super_core/fsm.cpp',
        )
        ros_path = os.path.join(
            repository,
            'planner/super_planner/super_planner_od/include/'
            'ros_interface/ros1/fsm_ros1.hpp',
        )
        with open(core_path, encoding='utf-8') as source_file:
            core = source_file.read()
        with open(ros_path, encoding='utf-8') as source_file:
            ros_integration = source_file.read()
        native_mission_path = os.path.join(
            workspace,
            'SUPER/mission_planner/include/waypoint_mission/'
            'ros1_waypoint_planner.hpp',
        )
        if not os.path.isfile(native_mission_path):
            self.skipTest('Optional upstream SUPER reference checkout is not present')
        with open(native_mission_path, encoding='utf-8') as source_file:
            native_mission = source_file.read()

        self.assertIn('GOAL_TERMINAL_STATE::REACHED', core)
        self.assertIn('GOAL_TERMINAL_STATE::INVALID', core)
        self.assertIn('GOAL_TERMINAL_STATE::FAILED', core)
        self.assertIn('goal_terminal_reported_.exchange(true)', core)
        self.assertIn('closeToGoal(0.1)', core)
        self.assertIn('handleTrajectoryFinished("PubCmdCallback")', ros_integration)
        self.assertNotIn('current_waypoint_reached_', ros_integration)
        self.assertNotIn('has_current_waypoint_terminal_', ros_integration)
        self.assertIn('cfg_.mission_waypoint_switch_distance', ros_integration)
        self.assertIn(
            '(position - cur_position).norm() < cfg_.switch_dis_vec[id]',
            native_mission,
        )

        command_callback = ros_integration.split(
            'void pubCmdTimerCallback', 1)[1].split(
                'void replanTimerCallback', 1)[0]
        self.assertNotIn('closeToGoal(', command_callback)

        queue_advance = ros_integration.split(
            'void advancePresetMission', 1)[1].split('void init(', 1)[0]
        self.assertNotIn('closeToGoal(', queue_advance)
        self.assertNotIn('traj_finish_', queue_advance)
        self.assertIn('(active_position - robot_state_.p).norm()', queue_advance)
        self.assertNotIn('machine_state_ != INIT', queue_advance)
        self.assertIn('MissionState::STATUS_FAILED', queue_advance)

    def test_super_mission_switch_distance_is_planner_owned(self):
        launch = get_planner_launch(
            'super_planner_od',
            {'mission_waypoint_switch_distance': '0.75'},
        )
        self.assertEqual(
            launch['args']['mission_waypoint_switch_distance'], '0.75')
        self.assertEqual(
            launch['runtime_parameter_expectations'][
                '/super_planner_od_fsm/mission/waypoint_switch_distance'
            ],
            0.75,
        )
        with self.assertRaises(ValueError):
            get_planner_launch(
                'super_planner_od',
                {'mission_waypoint_switch_distance': '0'},
            )


if __name__ == '__main__':
    unittest.main()
