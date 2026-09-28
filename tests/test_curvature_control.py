"""Tests for bounded curvature-aware steering and speed control."""

import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from curvature_control import CurvatureAwareControl  # noqa: E402


class StubAnalyzer:
    def __init__(
        self,
        preview_steering,
        upcoming_curvature,
        preview_curvature=None,
    ):
        self.preview_steering = preview_steering
        self.upcoming_curvature = upcoming_curvature
        self.preview_curvature = (
            upcoming_curvature
            if preview_curvature is None
            else preview_curvature
        )

    def metrics(self, waypoint_index):
        return {
            "preview_required_steering": self.preview_steering,
            "preview_curvature": self.preview_curvature,
            "upcoming_curvature": self.upcoming_curvature,
        }


class CurvatureAwareControlTests(unittest.TestCase):
    def test_feedforward_is_filtered_and_output_is_bounded(self):
        controller = CurvatureAwareControl(
            analyzer=StubAnalyzer(np.deg2rad(22.0), 1.6),
            feedforward_gain=0.85,
        )
        first = controller.update(
            waypoint_index=0,
            heading_error=np.deg2rad(40.0),
            cross_track_error=0.1,
            speed=0.2,
            dt=0.01,
        )
        self.assertLessEqual(abs(first), np.pi / 6)
        self.assertLess(
            abs(controller.filtered_feedforward),
            0.75 * np.deg2rad(22.0),
        )

    def test_curve_speed_reduces_smoothly_with_curvature(self):
        straight = CurvatureAwareControl(
            analyzer=StubAnalyzer(0.0, 0.0),
        )
        tight = CurvatureAwareControl(
            analyzer=StubAnalyzer(np.deg2rad(22.0), 1.6),
        )
        self.assertAlmostEqual(straight.recommended_speed(0.25, 0), 0.25)
        self.assertAlmostEqual(tight.recommended_speed(0.25, 0), 0.18)

    def test_heading_gain_reduces_only_for_tight_curvature(self):
        moderate = CurvatureAwareControl(
            analyzer=StubAnalyzer(np.deg2rad(16.0), 1.12, 1.12),
        )
        tight = CurvatureAwareControl(
            analyzer=StubAnalyzer(np.deg2rad(22.0), 1.6, 1.6),
        )
        moderate.update(0, 0.1, -0.02, 0.2, 0.01)
        tight.update(0, 0.1, -0.02, 0.2, 0.01)
        self.assertGreater(moderate.active_heading_gain, 0.65)
        self.assertAlmostEqual(tight.active_heading_gain, 0.55)


if __name__ == "__main__":
    unittest.main()
