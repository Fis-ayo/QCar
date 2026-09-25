"""Measurement-only front-camera lane-marking detection.

This module intentionally produces observations only.  It does not generate
steering commands.  Keeping image processing independent makes it possible to
validate the camera signal against the map/EKF controller before fusion.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LaneMeasurement:
    detected: bool
    confidence: float
    left_x_px: float
    right_x_px: float
    lane_center_x_px: float
    image_center_x_px: float
    center_offset_px: float
    center_offset_normalized: float
    heading_error_rad: float
    yellow_pixels: int
    white_pixels: int
    roi_top_px: int
    roi_bottom_px: int
    yellow_detected: bool
    yellow_confidence: float
    yellow_near_x_px: float
    yellow_far_x_px: float
    yellow_offset_normalized: float
    yellow_heading_rad: float
    yellow_curvature: float
    yellow_row_coverage: float
    yellow_fit_rmse_px: float
    yellow_temporal_jump_px: float
    yellow_near_y_px: float
    yellow_far_y_px: float
    yellow_extrapolated: bool
    perspective_calibrated: bool


class FrontLaneDetector:
    """Detect visible yellow/white lane-boundary candidates in a front image."""

    def __init__(
        self,
        roi_top_fraction=0.58,
        roi_bottom_fraction=0.93,
        minimum_band_pixels=30,
        minimum_lane_width_fraction=0.15,
        maximum_lane_width_fraction=0.90,
    ):
        self.roi_top_fraction = float(roi_top_fraction)
        self.roi_bottom_fraction = float(roi_bottom_fraction)
        self.minimum_band_pixels = int(minimum_band_pixels)
        self.minimum_lane_width_fraction = float(minimum_lane_width_fraction)
        self.maximum_lane_width_fraction = float(maximum_lane_width_fraction)
        self.previous_yellow_near_x = None
        self.last_yellow_line_mask = None

    def _yellow_geometry(self, yellow):
        """Fit a curved line to the best thin, vertically continuous component."""
        height, width = yellow.shape
        count, labels, stats, _ = cv2.connectedComponentsWithStats(yellow, 8)
        best = None
        for label in range(1, count):
            x, y, w, h, area = stats[label]
            if area < 100 or h < 0.18 * height:
                continue
            fill_ratio = area / max(1.0, float(w * h))
            if fill_ratio > 0.55 or w > 0.80 * width:
                continue
            score = h * np.sqrt(float(area)) * (1.0 - 0.5 * fill_ratio)
            if best is None or score > best[0]:
                best = (score, label)

        empty = {
            "detected": False, "confidence": 0.0,
            "near_x": float("nan"), "far_x": float("nan"),
            "offset": float("nan"), "heading": float("nan"),
            "curvature": float("nan"), "coverage": 0.0,
            "rmse": float("nan"), "jump": float("nan"),
            "near_y": float("nan"), "far_y": float("nan"),
            "extrapolated": False,
        }
        line_mask = np.zeros_like(yellow)
        if best is None:
            self.last_yellow_line_mask = line_mask
            return empty

        line_mask[labels == best[1]] = 255
        rows = []
        row_x = []
        for row in range(height):
            xs = np.flatnonzero(line_mask[row])
            if xs.size:
                rows.append(row)
                row_x.append(float(np.median(xs)))
        if len(rows) < max(12, int(0.12 * height)):
            self.last_yellow_line_mask = line_mask
            return empty

        rows = np.asarray(rows, dtype=float)
        row_x = np.asarray(row_x, dtype=float)
        y_normalized = rows / max(1.0, height - 1.0)
        x_normalized = row_x / max(1.0, width - 1.0)
        coefficients = np.polyfit(y_normalized, x_normalized, 2)
        fitted_x = np.polyval(coefficients, y_normalized) * (width - 1.0)
        rmse = float(np.sqrt(np.mean((row_x - fitted_x) ** 2)))

        # Evaluate only inside the rows where the component was observed.
        # Fixed-row evaluation previously extrapolated tight curves beyond the
        # image while still producing a high curve-fit confidence.
        near_row = float(np.percentile(rows, 90))
        far_row = float(np.percentile(rows, 20))
        near_y = near_row / max(1.0, height - 1.0)
        far_y = far_row / max(1.0, height - 1.0)
        near_x = float(np.polyval(coefficients, near_y) * (width - 1.0))
        far_x = float(np.polyval(coefficients, far_y) * (width - 1.0))
        coverage = float(len(np.unique(rows.astype(int))) / max(1.0, height))
        jump = (
            float("nan") if self.previous_yellow_near_x is None
            else abs(near_x - self.previous_yellow_near_x)
        )
        temporal_score = 1.0 if not np.isfinite(jump) else float(
            np.exp(-jump / max(1.0, 0.12 * width))
        )
        coverage_score = min(1.0, coverage / 0.55)
        fit_score = float(np.exp(-rmse / max(1.0, 0.025 * width)))
        confidence = float(coverage_score * fit_score * temporal_score)
        extrapolated = bool(
            near_x < 0.0 or near_x >= width or far_x < 0.0 or far_x >= width
        )
        detected = bool(
            confidence >= 0.20
            and not extrapolated
            and near_row > far_row
        )
        if detected:
            self.previous_yellow_near_x = near_x

        self.last_yellow_line_mask = line_mask
        return {
            "detected": detected,
            "confidence": confidence if detected else 0.0,
            "near_x": near_x,
            "far_x": far_x,
            "offset": (near_x - 0.5 * width) / max(1.0, 0.5 * width),
            # This is an image-space angle, not a metric road heading.  It is
            # retained for offline comparison until camera calibration exists.
            "heading": float(np.arctan2(near_x - far_x, near_row - far_row)),
            "curvature": float(coefficients[0]),
            "coverage": coverage,
            "rmse": rmse,
            "jump": jump,
            "near_y": near_row,
            "far_y": far_row,
            "extrapolated": extrapolated,
        }

    @staticmethod
    def _centroid_x(mask, y_start, y_end, side_start, side_end):
        band = mask[y_start:y_end, side_start:side_end]
        ys, xs = np.nonzero(band)
        if xs.size == 0:
            return float("nan"), 0
        return float(np.median(xs) + side_start), int(xs.size)

    def process(self, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("front-camera frame must be a BGR HxWx3 image")

        height, width = frame.shape[:2]
        roi_top = int(np.clip(self.roi_top_fraction * height, 0, height - 1))
        roi_bottom = int(
            np.clip(self.roi_bottom_fraction * height, roi_top + 1, height)
        )
        roi = frame[roi_top:roi_bottom]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        yellow = cv2.inRange(
            hsv,
            np.array([10, 50, 100], dtype=np.uint8),
            np.array([45, 255, 255], dtype=np.uint8),
        )
        white = cv2.inRange(
            hsv,
            np.array([0, 0, 160], dtype=np.uint8),
            np.array([180, 90, 255], dtype=np.uint8),
        )
        kernel = np.ones((3, 3), dtype=np.uint8)
        combined = cv2.morphologyEx(
            cv2.bitwise_or(yellow, white), cv2.MORPH_OPEN, kernel
        )
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        yellow_geometry = self._yellow_geometry(yellow)

        roi_height = combined.shape[0]
        near_start = int(0.62 * roi_height)
        far_end = max(1, int(0.38 * roi_height))
        center = width // 2

        left_near, left_count = self._centroid_x(
            combined, near_start, roi_height, 0, center
        )
        right_near, right_count = self._centroid_x(
            combined, near_start, roi_height, center, width
        )
        left_far, left_far_count = self._centroid_x(
            combined, 0, far_end, 0, center
        )
        right_far, right_far_count = self._centroid_x(
            combined, 0, far_end, center, width
        )

        lane_width = right_near - left_near
        width_valid = (
            np.isfinite(lane_width)
            and self.minimum_lane_width_fraction * width <= lane_width
            <= self.maximum_lane_width_fraction * width
        )
        counts_valid = (
            left_count >= self.minimum_band_pixels
            and right_count >= self.minimum_band_pixels
        )
        detected = bool(width_valid and counts_valid)

        if detected:
            lane_center = 0.5 * (left_near + right_near)
            center_offset = lane_center - 0.5 * width
            center_offset_normalized = center_offset / max(1.0, 0.5 * lane_width)
        else:
            lane_center = float("nan")
            center_offset = float("nan")
            center_offset_normalized = float("nan")

        far_valid = (
            left_far_count >= self.minimum_band_pixels
            and right_far_count >= self.minimum_band_pixels
        )
        if detected and far_valid:
            far_center = 0.5 * (left_far + right_far)
            vertical_pixels = max(1.0, near_start - 0.5 * far_end)
            heading_error = float(np.arctan2(lane_center - far_center, vertical_pixels))
        else:
            heading_error = float("nan")

        pixel_score = min(1.0, (left_count + right_count) / max(1.0, 0.08 * roi.size))
        far_score = 1.0 if far_valid else 0.0
        confidence = float((0.75 * pixel_score + 0.25 * far_score) if detected else 0.0)

        measurement = LaneMeasurement(
            detected=detected,
            confidence=confidence,
            left_x_px=left_near,
            right_x_px=right_near,
            lane_center_x_px=lane_center,
            image_center_x_px=0.5 * width,
            center_offset_px=center_offset,
            center_offset_normalized=center_offset_normalized,
            heading_error_rad=heading_error,
            yellow_pixels=int(np.count_nonzero(yellow)),
            white_pixels=int(np.count_nonzero(white)),
            roi_top_px=roi_top,
            roi_bottom_px=roi_bottom,
            yellow_detected=yellow_geometry["detected"],
            yellow_confidence=yellow_geometry["confidence"],
            yellow_near_x_px=yellow_geometry["near_x"],
            yellow_far_x_px=yellow_geometry["far_x"],
            yellow_offset_normalized=yellow_geometry["offset"],
            yellow_heading_rad=yellow_geometry["heading"],
            yellow_curvature=yellow_geometry["curvature"],
            yellow_row_coverage=yellow_geometry["coverage"],
            yellow_fit_rmse_px=yellow_geometry["rmse"],
            yellow_temporal_jump_px=yellow_geometry["jump"],
            yellow_near_y_px=yellow_geometry["near_y"],
            yellow_far_y_px=yellow_geometry["far_y"],
            yellow_extrapolated=yellow_geometry["extrapolated"],
            # QCarCameras exposes images but no CSI intrinsics/distortion
            # coefficients.  Do not treat image angle as road heading.
            perspective_calibrated=False,
        )
        return measurement, combined

    @staticmethod
    def render(frame, measurement, mask):
        output = frame.copy()
        top = measurement.roi_top_px
        bottom = measurement.roi_bottom_px
        width = output.shape[1]
        overlay = np.zeros_like(output[top:bottom])
        overlay[:, :, 1] = mask
        output[top:bottom] = cv2.addWeighted(
            output[top:bottom], 1.0, overlay, 0.35, 0.0
        )
        yellow_line = getattr(measurement, "yellow_detected", False)
        cv2.rectangle(output, (0, top), (width - 1, bottom - 1), (255, 180, 0), 2)
        if measurement.detected:
            y = bottom - 12
            for x, color in (
                (measurement.left_x_px, (0, 255, 255)),
                (measurement.right_x_px, (255, 255, 255)),
                (measurement.lane_center_x_px, (0, 255, 0)),
            ):
                cv2.circle(output, (int(round(x)), y), 7, color, -1)
        cv2.line(
            output,
            (int(round(measurement.image_center_x_px)), top),
            (int(round(measurement.image_center_x_px)), bottom),
            (255, 0, 255),
            2,
        )
        if yellow_line:
            near_y = int(round(top + measurement.yellow_near_y_px))
            far_y = int(round(top + measurement.yellow_far_y_px))
            cv2.line(
                output,
                (int(round(measurement.yellow_far_x_px)), far_y),
                (int(round(measurement.yellow_near_x_px)), near_y),
                (255, 0, 0),
                4,
            )
        label = (
            f"lane={int(measurement.detected)} "
            f"confidence={measurement.confidence:.2f} "
            f"offset={measurement.center_offset_normalized:.3f}"
        )
        cv2.putText(output, label, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if measurement.detected else (0, 0, 255), 2)
        yellow_label = (
            f"yellow={int(measurement.yellow_detected)} "
            f"confidence={measurement.yellow_confidence:.2f} "
            f"offset={measurement.yellow_offset_normalized:.3f}"
        )
        cv2.putText(
            output, yellow_label, (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (255, 0, 0) if measurement.yellow_detected else (0, 0, 255), 2,
        )
        return output
