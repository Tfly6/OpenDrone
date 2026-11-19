import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from .audit import matching_frame, ordered_visitation
from .validation import validate
from .runtime import RevisionFlightRunner as FlightRunner
from flight_eval.runner import RunnerState


class RevisionAuditTests(unittest.TestCase):
    def test_analytical_labels_and_ablations(self):
        with tempfile.TemporaryDirectory() as directory:
            validate(directory)
            self.assertTrue((Path(directory) / "fixtures.json").is_file())

    def test_sampling_does_not_invent_a_visit_between_states(self):
        audit = ordered_visitation([[0, 0, 0], [4, 0, 0]], [0, 1], [[2, 0, 0]], 0.1)
        self.assertFalse(audit["complete"])

    def test_missing_frame_is_not_accepted_as_identity(self):
        self.assertFalse(matching_frame("", "map", {}))
        self.assertFalse(matching_frame("odom", "map", {"world": "map"}))
        self.assertTrue(matching_frame("world", "map", {"world": "map"}))

    def test_runtime_contact_has_separate_clocks(self):
        runner = FlightRunner("se3_hopf", task_name="sequential_goal_mission", planner_name="ego_planner")
        runner._runner_state = RunnerState.EXECUTING
        runner._task_time_source = lambda: 12.5
        runner._task_clock_name = "ros"
        runner._on_contact_pulse(SimpleNamespace(other_collision="tree", self_collision="iris", point_count=1), now=900)
        event = runner._collision_metadata()["events"][0]
        self.assertEqual(event["time"], 900)
        self.assertEqual(event["task_time"], 12.5)
        self.assertEqual(event["task_clock"], "ros")
        self.assertIsNone(event["message_time"])

    def test_runner_transition_records_task_time(self):
        runner = FlightRunner("se3_hopf", task_name="hover", planner_name="none")
        runner._task_time_source = lambda: 10.0
        runner._task_clock_name = "ros"
        runner._transition(RunnerState.RECORDING, "test")
        self.assertEqual(runner._lifecycle_metadata()["transitions"][-1]["task_time"], 10.0)


if __name__ == "__main__":
    unittest.main()
