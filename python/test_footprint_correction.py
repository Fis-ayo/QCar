import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from footprint_correction import FootprintReferenceCorrection  # noqa: E402


class FootprintReferenceCorrectionTests(unittest.TestCase):
    def test_straight_path_has_no_correction(self):
        correction = FootprintReferenceCorrection()
        self.assertEqual(correction.target_for_curvature(0.0), 0.0)
        self.assertEqual(correction.target_for_curvature(0.05), 0.0)

    def test_left_and_right_turns_are_symmetric_and_outward(self):
        correction = FootprintReferenceCorrection()
        left = correction.target_for_curvature(1.122)
        right = correction.target_for_curvature(-1.122)
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertAlmostEqual(abs(left), abs(right), places=9)
        self.assertAlmostEqual(abs(left), 0.03604, places=4)

    def test_offset_is_bounded(self):
        correction = FootprintReferenceCorrection(maximum_offset=0.04)
        self.assertAlmostEqual(
            abs(correction.target_for_curvature(10.0)), 0.04
        )

    def test_gain_scales_kinematic_offset_before_cap(self):
        baseline = FootprintReferenceCorrection(
            correction_gain=1.0, maximum_offset=0.08
        )
        amplified = FootprintReferenceCorrection(
            correction_gain=1.65, maximum_offset=0.08
        )
        self.assertAlmostEqual(
            abs(amplified.target_for_curvature(1.122)),
            1.65 * abs(baseline.target_for_curvature(1.122)),
        )

    def test_applied_offset_is_smoothed_and_rate_limited(self):
        correction = FootprintReferenceCorrection(
            time_constant=0.20,
            maximum_rate=0.12,
        )
        first = correction.update(1.122, 0.01)
        self.assertLess(first, 0.0)
        self.assertLessEqual(abs(first), 0.0012 + 1e-12)
        for _ in range(500):
            final = correction.update(1.122, 0.01)
        self.assertAlmostEqual(
            final, correction.target_for_curvature(1.122), places=5
        )


if __name__ == "__main__":
    unittest.main()
