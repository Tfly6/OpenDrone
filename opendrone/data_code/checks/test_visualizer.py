#!/usr/bin/env python3

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

from flight_eval.visualizer import BagVisualizer


class CorePlotTest(unittest.TestCase):
    def test_single_point_planner_stream_uses_one_scatter_artist(self):
        axis = MagicMock()
        segments = [
            np.array([[float(index), 0.0, 2.0]])
            for index in range(10100)
        ]

        BagVisualizer._draw_planner_segments(axis, segments)

        axis.scatter.assert_called_once()
        self.assertEqual(len(axis.scatter.call_args.args[0]), 10100)
        axis.add_collection.assert_not_called()

    def test_horizon_windows_use_one_3d_collection_artist(self):
        axis = MagicMock()
        segments = [
            np.array([
                [float(index), 0.0, 2.0],
                [float(index) + 1.0, 1.0, 2.0],
            ])
            for index in range(1000)
        ]

        BagVisualizer._draw_planner_segments(axis, segments, is_3d=True)

        axis.add_collection3d.assert_called_once()
        axis.scatter.assert_not_called()

    def test_automatic_core_plots_include_metrics_overview(self):
        visualizer = BagVisualizer.__new__(BagVisualizer)
        visualizer.analyzer = SimpleNamespace(
            task=SimpleNamespace(render_trajectory_plots=True),
        )

        with patch.object(
            visualizer,
            '_save_trajectory_plots',
            return_value={'trajectory_xy': 'trajectory_xy.png'},
        ), patch.object(
            visualizer,
            '_save_state_plots',
            return_value={'position_tracking': 'position_tracking.png'},
        ), patch.object(
            visualizer,
            '_save_yaw_tracking_plot',
            return_value={'yaw_tracking_error': 'yaw_tracking_error.png'},
        ), patch.object(
            visualizer,
            '_save_metrics_overview',
            return_value={'metrics_overview': 'metrics_overview.png'},
        ) as metrics_overview:
            artifacts = visualizer._save_core_plots(
                object(),
                {},
                '/tmp/plots',
                220,
                report=object(),
            )

        self.assertEqual(artifacts['metrics_overview'], 'metrics_overview.png')
        metrics_overview.assert_called_once()


if __name__ == '__main__':
    unittest.main()
