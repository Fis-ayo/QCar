"""Tests for route-independent smooth polyline shadow tracking."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from path_tracking import SmoothPolylineTracker  # noqa: E402


class SmoothPolylineTrackerTests(unittest.TestCase):
    def test_signed_cross_track_is_continuous_across_straight_vertices(self):
        waypoints = np.array([
            [0.0, 0.1, 0.2, 0.3, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ])
        tracker = SmoothPolylineTracker(waypoints)
        before = tracker.update([0.099, 0.05], 0.0, 0.2, 0)
        after = tracker.update([0.101, 0.05], 0.0, 0.2, 1)
        self.assertTrue(before["available"])
        self.assertAlmostEqual(before["cross_track_error_m"], -0.05, places=3)
        self.assertAlmostEqual(
            before["cross_track_error_m"],
            after["cross_track_error_m"],
            places=3,
        )

    def test_low_speed_regularization_prevents_unbounded_feedback(self):
        waypoints = np.array([
            [0.0, 0.2, 0.4, 0.6, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ])
        tracker = SmoothPolylineTracker(waypoints)
        result = tracker.update([0.1, 0.05], 0.0, 0.0, 0)
        self.assertAlmostEqual(result["control_speed_mps"], 0.15)
        self.assertLess(abs(result["feedback_steering_rad"]), math.pi / 2)

    def test_heading_filter_rejects_opposite_direction_lane(self):
        waypoints = np.array([
            [0.0, 1.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.1, 0.0],
        ])
        tracker = SmoothPolylineTracker(waypoints)
        result = tracker.update([0.5, 0.05], 0.0, 0.2, 0)
        self.assertTrue(result["available"])
        self.assertNotEqual(result["segment_index"], 2)
        self.assertEqual(result["authority"], 0.0)


if __name__ == "__main__":
    unittest.main()
