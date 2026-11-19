"""Task-clock instrumentation and launch configuration for evaluation runs."""
from pathlib import Path

from flight_eval.runner import FlightRunner
from flight_eval.planners import PLANNER_REGISTRY
from .paths import workspace_root


class RevisionFlightRunner(FlightRunner):
    def _transition(self, next_state, reason=''):
        with self._state_condition:
            count = len(self._state_history)
            super()._transition(next_state, reason)
            if len(self._state_history) > count:
                self._state_history[-1].update(task_time=self._task_now(), task_clock=self._task_clock_name)

    def _on_contact_pulse(self, msg, now=None):
        with self._state_condition:
            count = len(self._collision_events)
            task_time = self._task_now()
            super()._on_contact_pulse(msg, now)
            if len(self._collision_events) > count:
                self._collision_events[-1].update(
                    task_time=task_time, task_clock=self._task_clock_name,
                    message_time=msg.header.stamp.to_sec() if hasattr(msg, 'header') else None)


def configure_revision_runtime():
    """Configure the active experiment process and its task-clock recorder."""
    import flight_eval.batch
    import flight_eval.environment
    # The experiment workspace can be configured independently of checkout name.
    flight_eval.batch._WORKSPACE_ROOT = str(workspace_root())
    flight_eval.environment._WORKSPACE_ROOT = str(workspace_root())
    flight_eval.batch.FlightRunner = RevisionFlightRunner
    launch_dir = Path(__file__).resolve().parent / 'launch'
    for name, filename in (
        ('ego_planner', 'benchmark_ego_planner.launch'),
        ('fast_planner_kino', 'benchmark_fast_planner.launch'),
    ):
        PLANNER_REGISTRY[name]['launch_file'] = str(launch_dir / filename)
