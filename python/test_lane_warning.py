import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_warning import LaneBoundaryWarning  # noqa: E402


class LaneBoundaryWarningTests(unittest.TestCase):
    def test_warning_requires_sustained_near_measurements(self):
        warning = LaneBoundaryWarning(enter_frames=3)
        self.assertFalse(warning.update(True, 0.8, -0.2))
        self.assertFalse(warning.update(True, 0.8, -0.2))
        self.assertTrue(warning.update(True, 0.8, -0.2))

    def test_low_confidence_does_not_enter_warning(self):
        warning = LaneBoundaryWarning(enter_frames=2)
        self.assertFalse(warning.update(True, 0.4, -0.1))
        self.assertFalse(warning.update(True, 0.4, -0.1))

    def test_hysteresis_prevents_chatter(self):
        warning = LaneBoundaryWarning(
            enter_frames=1, exit_frames=2, latch_after_entry=False
        )
        self.assertTrue(warning.update(True, 0.9, -0.2))
        self.assertTrue(warning.update(True, 0.9, -0.36))
        self.assertTrue(warning.update(True, 0.9, -0.5))
        self.assertFalse(warning.update(True, 0.9, -0.5))

    def test_missing_detection_clears_only_after_exit_streak(self):
        warning = LaneBoundaryWarning(
            enter_frames=1, exit_frames=2, latch_after_entry=False
        )
        self.assertTrue(warning.update(True, 0.9, -0.2))
        self.assertTrue(warning.update(False, 0.0, 0.0))
        self.assertFalse(warning.update(False, 0.0, 0.0))

    def test_default_warning_latches_for_run_record(self):
        warning = LaneBoundaryWarning(enter_frames=1)
        self.assertTrue(warning.update(True, 0.9, -0.2))
        self.assertTrue(warning.update(False, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
