"""Local tests for measurement-only front-camera lane detection."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_detection import FrontLaneDetector  # noqa: E402


class FrontLaneDetectorTests(unittest.TestCase):
    def setUp(self):
        self.detector = FrontLaneDetector(
            roi_top_fraction=0.4,
            roi_bottom_fraction=1.0,
            minimum_band_pixels=10,
        )

    def test_detects_centered_yellow_and_white_boundaries(self):
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.line(frame, (90, 239), (140, 96), (0, 255, 255), 12)
        cv2.line(frame, (310, 239), (260, 96), (255, 255, 255), 12)

        measurement, mask = self.detector.process(frame)

        self.assertTrue(measurement.detected)
        self.assertAlmostEqual(measurement.center_offset_normalized, 0.0, delta=0.08)
        self.assertGreater(measurement.yellow_pixels, 0)
        self.assertGreater(measurement.white_pixels, 0)
        self.assertGreater(np.count_nonzero(mask), 0)
        self.assertTrue(measurement.yellow_detected)
        self.assertGreater(measurement.yellow_confidence, 0.0)
        self.assertTrue(np.isfinite(measurement.yellow_heading_rad))

    def test_rejects_frame_without_two_boundaries(self):
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.line(frame, (90, 239), (140, 96), (0, 255, 255), 12)

        measurement, _ = self.detector.process(frame)

        self.assertFalse(measurement.detected)
        self.assertTrue(np.isnan(measurement.center_offset_normalized))
        self.assertTrue(measurement.yellow_detected)

    def test_rejects_large_filled_yellow_region_as_line(self):
        frame = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.rectangle(frame, (20, 100), (380, 239), (0, 255, 255), -1)

        measurement, _ = self.detector.process(frame)

        self.assertFalse(measurement.yellow_detected)

    def test_temporal_jump_reduces_confidence(self):
        centered = np.zeros((240, 400, 3), dtype=np.uint8)
        shifted = np.zeros((240, 400, 3), dtype=np.uint8)
        cv2.line(centered, (100, 239), (140, 96), (0, 255, 255), 10)
        cv2.line(shifted, (300, 239), (340, 96), (0, 255, 255), 10)

        first, _ = self.detector.process(centered)
        second, _ = self.detector.process(shifted)

        self.assertTrue(first.yellow_detected)
        self.assertGreater(first.yellow_confidence, second.yellow_confidence)
        self.assertGreater(second.yellow_temporal_jump_px, 100.0)

    def test_rejects_invalid_frame(self):
        with self.assertRaises(ValueError):
            self.detector.process(np.zeros((10, 10), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
