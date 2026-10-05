"""Tests for explicit experimental actuator-authority policies."""

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from control_modes import get_control_mode  # noqa: E402


class ControlModeTests(unittest.TestCase):
    def test_baseline_restores_original_control_authority(self):
        mode = get_control_mode("baseline")
        self.assertFalse(mode.safe_speed_control)
        self.assertFalse(mode.regularized_stanley_speed)
        self.assertFalse(mode.localization_stop)
        self.assertFalse(mode.tracking_loss_stop)
        self.assertFalse(mode.camera_steering)
        self.assertFalse(mode.boundary_speed_control)

    def test_baseline_safety_never_enables_camera(self):
        mode = get_control_mode("baseline_safety")
        self.assertTrue(mode.safe_speed_control)
        self.assertTrue(mode.localization_stop)
        self.assertTrue(mode.tracking_loss_stop)
        self.assertFalse(mode.camera_steering)
        self.assertFalse(mode.boundary_speed_control)

    def test_preventive_stop_has_speed_only_camera_safety(self):
        mode = get_control_mode("preventive_stop")
        self.assertTrue(mode.safe_speed_control)
        self.assertTrue(mode.boundary_speed_control)
        self.assertFalse(mode.camera_steering)

    def test_hybrid_has_one_camera_authority_path(self):
        mode = get_control_mode("hybrid")
        self.assertTrue(mode.camera_steering)
        self.assertFalse(mode.boundary_speed_control)

    def test_ab_modes_have_identical_non_camera_behavior(self):
        baseline = get_control_mode("baseline_safety")
        hybrid = get_control_mode("hybrid")
        self.assertEqual(
            baseline.non_camera_signature,
            hybrid.non_camera_signature,
        )

    def test_unknown_mode_fails(self):
        with self.assertRaises(ValueError):
            get_control_mode("experimental-mystery")


if __name__ == "__main__":
    unittest.main()
