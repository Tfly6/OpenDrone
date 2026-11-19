#!/usr/bin/env python3

import os
import signal
import tempfile
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from flight_eval.batch import (
    BatchConfigError,
    _BatchTerminationGuard,
    _ManagedProcess,
    _MavrosStateObserver,
    _SimulationSession,
    load_batch_definition,
)


class SimulationSessionTest(unittest.TestCase):
    class _FakeStateObserver:
        sequence = 7

        def matches(self, connected, after_sequence, max_age):
            del max_age
            return connected is True and after_sequence == 7

    def test_px4_starts_without_interactive_pxh(self):
        session = _SimulationSession({}, '', '')
        session.px4_build_dir = '/tmp/px4-build'
        session._mavros_state = self._FakeStateObserver()
        managed_process = object()

        with tempfile.TemporaryDirectory() as temp_dir:
            working_dir = os.path.join(temp_dir, 'px4-rootfs')
            with ExitStack() as stack:
                stack.enter_context(patch.object(
                    session,
                    '_prepare_px4_data_dir',
                    return_value=('/tmp/px4-etc', 'etc/init.d-posix/rcS'),
                ))
                start_process = stack.enter_context(patch.object(
                    session,
                    '_start_process',
                    return_value=managed_process,
                ))
                stack.enter_context(patch.object(session, '_wait_for_px4_ready'))
                session.start_px4('px4.log', working_dir)

        command = start_process.call_args.args[1]
        self.assertEqual(
            command,
            [
                '/tmp/px4-build/bin/px4',
                '-d',
                '/tmp/px4-etc',
                '-s',
                'etc/init.d-posix/rcS',
            ],
        )
        self.assertIs(session.px4_process, managed_process)

    def test_mavros_state_observer_owns_one_reusable_subscription(self):
        class Subscriber:
            def __init__(self):
                self.unregistered = False

            def unregister(self):
                self.unregistered = True

        subscriber = Subscriber()
        rospy = type('Rospy', (), {
            'Subscriber': staticmethod(lambda *args, **kwargs: subscriber),
        })
        observer = _MavrosStateObserver(rospy, object)
        observer._on_state(type('State', (), {'connected': True})())

        self.assertTrue(observer.matches(True, 0, max_age=3.0))
        self.assertFalse(observer.matches(False, 0, max_age=3.0))
        observer.close()
        self.assertTrue(subscriber.unregistered)

    def test_mavros_link_checks_do_not_spawn_rostopic_processes(self):
        session = _SimulationSession({}, '', '')
        session._mavros_state = self._FakeStateObserver()
        session._mavros_transition_sequence = 7

        with patch('flight_eval.batch.subprocess.run') as run:
            self.assertTrue(session._mavros_connected())
            self.assertFalse(session._mavros_disconnected())

        run.assert_not_called()

    def test_px4_ready_does_not_probe_parameter_cache(self):
        session = _SimulationSession({}, '', '')
        session.px4_process = object()
        session.timeouts = {'px4': 60.0, 'px4_ready': 0.0}

        with patch.object(session, '_wait_until') as wait_until:
            session._wait_for_px4_ready()

        wait_until.assert_called_once()
        predicate = wait_until.call_args.args[0]
        self.assertIs(predicate.__self__, session)
        self.assertIs(predicate.__func__, session._mavros_connected.__func__)
        self.assertEqual(
            wait_until.call_args.args[2],
            'MAVROS 与新启动 PX4 的连接',
        )

    def test_managed_process_stops_group_after_leader_exits(self):
        process = SimpleNamespace(pid=1234, poll=lambda: 0, wait=lambda timeout: 0)
        managed = _ManagedProcess('test', process, '/tmp/test.log')

        with patch(
            'flight_eval.batch.os.killpg',
            side_effect=[None, ProcessLookupError],
        ) as killpg:
            stopped = managed.stop()

        self.assertTrue(stopped)
        killpg.assert_any_call(1234, signal.SIGINT)
        killpg.assert_any_call(1234, 0)

    def test_batch_termination_guard_ignores_repeated_signal_during_cleanup(self):
        guard = _BatchTerminationGuard()
        guard._original_handlers = {
            signal.SIGINT: signal.default_int_handler,
            signal.SIGTERM: signal.SIG_DFL,
        }
        with patch('flight_eval.batch.signal.signal') as set_handler:
            with self.assertRaises(KeyboardInterrupt):
                guard._handle(signal.SIGTERM, None)

        self.assertEqual(
            set_handler.call_count,
            len(guard._original_handlers),
        )

class BatchDefinitionTest(unittest.TestCase):
    @staticmethod
    def _write_config(directory, extra_experiment=''):
        path = os.path.join(directory, 'batch.yaml')
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write(
                'version: 1\n'
                'name: discrete_circle\n'
                'defaults:\n'
                '  simulation:\n'
                '    px4:\n'
                '      source_dir: /tmp/fake-px4\n'
                '    gazebo:\n'
                '      respawn_gazebo: true\n'
                '    mavros:\n'
                '      respawn_mavros: true\n'
                'experiments:\n'
                '  - name: discrete_circle\n'
                '    task: discrete_circle\n'
                '    controllers: [se3_hopf]\n'
                '    planners: [mav_trajectory]\n'
                f'{extra_experiment}'
            )
        return path

    def test_batch_directory_uses_top_level_name_not_planner_name(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = self._write_config(temp_dir)
            with patch(
                'flight_eval.batch.os.path.isdir',
                return_value=True,
            ), patch('flight_eval.batch.datetime') as mock_datetime:
                mock_datetime.now.return_value.strftime.return_value = '20260727_010203'
                definition = load_batch_definition(config_path)

        self.assertEqual(definition.name, 'discrete_circle')
        self.assertEqual(
            os.path.basename(definition.output_dir),
            'discrete_circle_20260727_010203',
        )

    def test_removed_repetitions_alias_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = self._write_config(
                temp_dir,
                '    repetitions: 2\n',
            )
            with self.assertRaisesRegex(BatchConfigError, 'repetitions'):
                load_batch_definition(config_path)

    def test_collision_episode_options_are_validated(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = self._write_config(
                temp_dir,
                '    max_collision_episodes: 5\n'
                '    collision_episode_gap: 0.8\n',
            )
            with patch('flight_eval.batch.os.path.isdir', return_value=True):
                definition = load_batch_definition(config_path)
        self.assertEqual(
            definition.cases[0].run_options['max_collision_episodes'],
            5,
        )
        self.assertEqual(
            definition.cases[0].run_options['collision_episode_gap'],
            0.8,
        )

    def test_collision_episode_limit_must_be_positive_integer(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = self._write_config(
                temp_dir,
                '    max_collision_episodes: 0\n',
            )
            with self.assertRaisesRegex(BatchConfigError, '正整数'):
                load_batch_definition(config_path)


if __name__ == '__main__':
    unittest.main()
