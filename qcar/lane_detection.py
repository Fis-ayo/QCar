"""Measurement-only front-camera lane-marking detection.

This module intentionally produces observations only.  It does not generate
steering commands.  Keeping image processing independent makes it possible to
validate the camera signal against the map/EKF controller before fusion.
"""

from dataclasses import dataclass
from collections import deque

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
    yellow_reference_detected: bool
    yellow_reference_x_px: float
    yellow_reference_y_px: float
    yellow_reference_confidence: float
    yellow_multirow_fractions: tuple
    yellow_multirow_detected: tuple
    yellow_multirow_x_px: tuple
    yellow_multirow_confidence: tuple
    corridor_detected: bool
    corridor_confidence: float
    corridor_paired_rows: int
    corridor_yellow_near_x_px: float
    corridor_white_near_x_px: float
    corridor_center_near_x_px: float
    corridor_offset_normalized: float
    corridor_heading_rad: float
    corridor_outer_source: str
    quanser_yellow_detected: bool
    quanser_yellow_confidence: float
    quanser_yellow_slope: float
    quanser_yellow_intercept: float
    quanser_yellow_fit_rmse_px: float
    quanser_shadow_steering_rad: float
    corridor_trend_state: str
    corridor_trend_offset_delta: float
    corridor_trend_heading_delta: float
    corridor_trend_history_count: int
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
        self.corridor_history = deque(maxlen=8)
        self.corridor_missing_frames = 0
        self.last_corridor_trend_state = "UNAVAILABLE"

    @staticmethod
    def _quanser_yellow_feature(frame):
        """Reproduce the Quanser CSI yellow slope/intercept measurement.

        Quanser's example fits ``crop_height - row`` as a linear function of
        the yellow-pixel column inside image rows 524:674 and columns 0:820 at
        1640x820 resolution.  This deterministic version uses all pixels.
        """
        height, width = frame.shape[:2]
        y0 = int(round(524.0 / 820.0 * height))
        y1 = int(round(674.0 / 820.0 * height))
        x1 = int(round(820.0 / 1640.0 * width))
        crop = frame[y0:y1, 0:x1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(
            hsv,
            np.array([10, 50, 100], dtype=np.uint8),
            np.array([45, 255, 255], dtype=np.uint8),
        )
        rows, cols = np.nonzero(yellow)
        if cols.size <= 1000:
            return {
                "detected": False, "confidence": 0.0,
                "slope": float("nan"), "intercept": float("nan"),
                "rmse": float("nan"), "shadow_steering": float("nan"),
            }
        y_values = yellow.shape[0] - rows.astype(float)
        coefficients = np.polyfit(cols.astype(float), y_values, 1)
        fitted = np.polyval(coefficients, cols.astype(float))
        rmse = float(np.sqrt(np.mean((y_values - fitted) ** 2)))
        slope, intercept = map(float, coefficients)
        fit_score = float(np.exp(-rmse / 20.0))
        pixel_score = min(1.0, cols.size / 5000.0)
        confidence = fit_score * pixel_score
        raw_steering = 1.5 * (slope - 0.3419) + (intercept + 5.0) / 150.0
        return {
            "detected": confidence >= 0.20,
            "confidence": confidence,
            "slope": slope,
            "intercept": intercept,
            "rmse": rmse,
            "shadow_steering": float(np.clip(raw_steering, -0.5, 0.5)),
        }

    def _corridor_trend(self, corridor):
        if not corridor["detected"]:
            self.corridor_missing_frames += 1
            state = (
                "OUTWARD_LOST"
                if self.last_corridor_trend_state == "OUTWARD_APPROACH"
                and self.corridor_missing_frames <= 5
                else "UNAVAILABLE"
            )
            self.last_corridor_trend_state = state
            return state, float("nan"), float("nan"), len(self.corridor_history)

        self.corridor_missing_frames = 0
        self.corridor_history.append((
            corridor["offset"], corridor["heading"]
        ))
        offset_delta = 0.0
        heading_delta = 0.0
        state = "STABLE"
        if len(self.corridor_history) >= 5:
            offset_delta = float(
                corridor["offset"] - self.corridor_history[0][0]
            )
            heading_delta = float(
                corridor["heading"] - self.corridor_history[0][1]
            )
            if offset_delta <= -0.08 and heading_delta <= 0.05:
                state = "OUTWARD_APPROACH"
            elif offset_delta >= 0.08 and heading_delta >= 0.05:
                state = "YELLOW_APPROACH"
        self.last_corridor_trend_state = state
        return state, offset_delta, heading_delta, len(self.corridor_history)

    @staticmethod
    def _yellow_at_fixed_row(yellow, row_fraction=0.85, half_band=5):
        """Measure observed yellow pixels at one repeatable image row.

        This intentionally uses the threshold mask rather than evaluating the
        polynomial outside a component's observed row range. The result is
        suitable for comparing stationary calibration poses because its image
        height never changes between frames.
        """
        height, width = yellow.shape
        reference_y = int(round(row_fraction * max(0, height - 1)))
        y0 = max(0, reference_y - int(half_band))
        y1 = min(height, reference_y + int(half_band) + 1)
        band = yellow[y0:y1]
        _, xs = np.nonzero(band)
        if xs.size < 20:
            return False, float("nan"), float(reference_y), 0.0

        # Split occupied columns into runs and retain the strongest compact
        # run. This rejects remote yellow-colored blobs without assuming which
        # side of the image contains the divider.
        counts = np.bincount(xs, minlength=width)
        occupied = np.flatnonzero(counts)
        splits = np.flatnonzero(np.diff(occupied) > 3) + 1
        runs = np.split(occupied, splits)
        candidates = []
        for run in runs:
            if run.size == 0:
                continue
            count = int(np.sum(counts[run]))
            span = int(run[-1] - run[0] + 1)
            if count >= 20 and span <= int(0.20 * width):
                candidates.append((count, run))
        if not candidates:
            return False, float("nan"), float(reference_y), 0.0

        count, run = max(candidates, key=lambda item: item[0])
        run_mask = np.isin(xs, run)
        reference_x = float(np.median(xs[run_mask]))
        density = count / max(1.0, float((y1 - y0) * len(run)))
        confidence = float(min(1.0, count / 80.0) * min(1.0, density / 0.35))
        return True, reference_x, float(reference_y), confidence

    @staticmethod
    def _compact_runs_at_row(mask, row_fraction, half_band=5):
        """Return compact colored runs observed at a fixed image row."""
        height, width = mask.shape
        reference_y = int(round(float(row_fraction) * max(0, height - 1)))
        y0 = max(0, reference_y - int(half_band))
        y1 = min(height, reference_y + int(half_band) + 1)
        _, xs = np.nonzero(mask[y0:y1])
        if xs.size < 20:
            return reference_y, []
        counts = np.bincount(xs, minlength=width)
        occupied = np.flatnonzero(counts)
        splits = np.flatnonzero(np.diff(occupied) > 3) + 1
        candidates = []
        for run in np.split(occupied, splits):
            if run.size == 0:
                continue
            count = int(np.sum(counts[run]))
            span = int(run[-1] - run[0] + 1)
            if count < 20 or span > int(0.20 * width):
                continue
            run_mask = np.isin(xs, run)
            center = float(np.median(xs[run_mask]))
            density = count / max(1.0, float((y1 - y0) * len(run)))
            confidence = float(
                min(1.0, count / 80.0) * min(1.0, density / 0.35)
            )
            candidates.append((center, confidence))
        return reference_y, candidates

    @staticmethod
    def _dark_road_edges(mask, row_fraction, half_band=5):
        """Return right edges of compact dark-road runs at one image row."""
        height, width = mask.shape
        reference_y = int(round(float(row_fraction) * max(0, height - 1)))
        y0 = max(0, reference_y - int(half_band))
        y1 = min(height, reference_y + int(half_band) + 1)
        coverage = np.mean(mask[y0:y1] > 0, axis=0)
        occupied = np.flatnonzero(coverage >= 0.45)
        if occupied.size == 0:
            return reference_y, []
        splits = np.flatnonzero(np.diff(occupied) > 3) + 1
        edges = []
        for run in np.split(occupied, splits):
            if run.size < int(0.08 * width):
                continue
            mean_coverage = float(np.mean(coverage[run]))
            edges.append((float(run[0]), float(run[-1]), mean_coverage))
        return reference_y, edges

    @classmethod
    def _lane_corridor(
        cls, yellow, white, dark_road, row_fractions=(0.35, 0.60, 0.85)
    ):
        """Pair the yellow divider with a physical outer road edge."""
        _, width = yellow.shape
        pairs = []
        for fraction in row_fractions:
            row_y, yellow_runs = cls._compact_runs_at_row(yellow, fraction)
            _, white_runs = cls._compact_runs_at_row(white, fraction)
            _, dark_runs = cls._dark_road_edges(dark_road, fraction)
            if not yellow_runs:
                continue
            yellow_x, yellow_confidence = max(
                yellow_runs, key=lambda item: item[1]
            )
            dark_candidates = [
                item for item in dark_runs
                if item[0] <= yellow_x + 0.12 * width
                and 0.12 * width <= item[1] - yellow_x <= 0.90 * width
                and item[1] < 0.95 * width
            ]
            white_candidates = [
                item for item in white_runs
                if 0.12 * width <= item[0] - yellow_x <= 0.90 * width
            ]
            if dark_candidates:
                _, outer_x, outer_confidence = max(
                    dark_candidates, key=lambda item: item[1]
                )
                source = "dark_road_edge"
            elif white_candidates:
                outer_x, outer_confidence = min(
                    white_candidates, key=lambda item: item[0]
                )
                source = "white_run"
            else:
                continue
            pairs.append({
                "fraction": float(fraction),
                "y": float(row_y),
                "yellow_x": yellow_x,
                "white_x": outer_x,
                "center_x": 0.5 * (yellow_x + outer_x),
                "width": outer_x - yellow_x,
                "confidence": min(yellow_confidence, outer_confidence),
                "source": source,
            })

        empty = {
            "detected": False, "confidence": 0.0, "paired_rows": len(pairs),
            "yellow_near_x": float("nan"), "white_near_x": float("nan"),
            "center_near_x": float("nan"), "offset": float("nan"),
            "heading": float("nan"),
            "source": "none",
        }
        if len(pairs) < 2:
            return empty
        lane_widths = np.asarray([pair["width"] for pair in pairs])
        width_variation = float(np.std(lane_widths) / max(1.0, np.mean(lane_widths)))
        if width_variation > 0.35:
            return empty
        pairs.sort(key=lambda pair: pair["fraction"])
        far = pairs[0]
        near = pairs[-1]
        vertical = max(1.0, near["y"] - far["y"])
        heading = float(np.arctan2(
            near["center_x"] - far["center_x"], vertical
        ))
        offset = float(
            (near["center_x"] - 0.5 * width)
            / max(1.0, 0.5 * near["width"])
        )
        confidence = float(
            np.mean([pair["confidence"] for pair in pairs])
            * np.exp(-2.0 * width_variation)
            * min(1.0, len(pairs) / 3.0)
        )
        return {
            "detected": confidence >= 0.30,
            "confidence": confidence,
            "paired_rows": len(pairs),
            "yellow_near_x": near["yellow_x"],
            "white_near_x": near["white_x"],
            "center_near_x": near["center_x"],
            "offset": offset,
            "heading": heading,
            "source": near["source"],
        }

    @classmethod
    def _yellow_at_multiple_rows(cls, yellow, row_fractions=(0.05, 0.50, 0.85)):
        """Sample the same yellow boundary at repeatable near/far image rows.

        Each row is measured independently from observed pixels.  No curve is
        extrapolated into an unobserved part of the image.
        """
        samples = [cls._yellow_at_fixed_row(yellow, fraction) for fraction in row_fractions]
        return {
            "fractions": tuple(float(value) for value in row_fractions),
            "detected": tuple(bool(sample[0]) for sample in samples),
            "x_px": tuple(float(sample[1]) for sample in samples),
            "confidence": tuple(float(sample[3]) for sample in samples),
        }

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
        quanser_yellow = self._quanser_yellow_feature(frame)
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
        dark_road = cv2.inRange(
            hsv,
            np.array([0, 0, 0], dtype=np.uint8),
            np.array([180, 255, 125], dtype=np.uint8),
        )
        dark_road = cv2.morphologyEx(
            dark_road, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8)
        )
        kernel = np.ones((3, 3), dtype=np.uint8)
        combined = cv2.morphologyEx(
            cv2.bitwise_or(yellow, white), cv2.MORPH_OPEN, kernel
        )
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        yellow_geometry = self._yellow_geometry(yellow)
        (
            yellow_reference_detected,
            yellow_reference_x,
            yellow_reference_y,
            yellow_reference_confidence,
        ) = self._yellow_at_fixed_row(yellow)
        yellow_multirow = self._yellow_at_multiple_rows(yellow)
        corridor = self._lane_corridor(yellow, white, dark_road)
        (
            corridor_trend_state,
            corridor_trend_offset_delta,
            corridor_trend_heading_delta,
            corridor_trend_history_count,
        ) = self._corridor_trend(corridor)

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
            yellow_reference_detected=yellow_reference_detected,
            yellow_reference_x_px=yellow_reference_x,
            yellow_reference_y_px=yellow_reference_y,
            yellow_reference_confidence=yellow_reference_confidence,
            yellow_multirow_fractions=yellow_multirow["fractions"],
            yellow_multirow_detected=yellow_multirow["detected"],
            yellow_multirow_x_px=yellow_multirow["x_px"],
            yellow_multirow_confidence=yellow_multirow["confidence"],
            corridor_detected=corridor["detected"],
            corridor_confidence=corridor["confidence"],
            corridor_paired_rows=corridor["paired_rows"],
            corridor_yellow_near_x_px=corridor["yellow_near_x"],
            corridor_white_near_x_px=corridor["white_near_x"],
            corridor_center_near_x_px=corridor["center_near_x"],
            corridor_offset_normalized=corridor["offset"],
            corridor_heading_rad=corridor["heading"],
            corridor_outer_source=corridor["source"],
            quanser_yellow_detected=quanser_yellow["detected"],
            quanser_yellow_confidence=quanser_yellow["confidence"],
            quanser_yellow_slope=quanser_yellow["slope"],
            quanser_yellow_intercept=quanser_yellow["intercept"],
            quanser_yellow_fit_rmse_px=quanser_yellow["rmse"],
            quanser_shadow_steering_rad=quanser_yellow["shadow_steering"],
            corridor_trend_state=corridor_trend_state,
            corridor_trend_offset_delta=corridor_trend_offset_delta,
            corridor_trend_heading_delta=corridor_trend_heading_delta,
            corridor_trend_history_count=corridor_trend_history_count,
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
        if measurement.yellow_reference_detected:
            reference_y = int(round(top + measurement.yellow_reference_y_px))
            reference_x = int(round(measurement.yellow_reference_x_px))
            cv2.circle(
                output, (reference_x, reference_y), 8, (0, 128, 255), -1
            )
        roi_height = bottom - top
        for fraction, detected, x_value in zip(
            measurement.yellow_multirow_fractions,
            measurement.yellow_multirow_detected,
            measurement.yellow_multirow_x_px,
        ):
            if not detected:
                continue
            row_y = int(round(top + fraction * max(0, roi_height - 1)))
            cv2.circle(output, (int(round(x_value)), row_y), 6, (255, 255, 0), 2)
        if measurement.corridor_detected:
            y = int(round(top + 0.85 * max(0, bottom - top - 1)))
            cv2.line(
                output,
                (int(round(measurement.corridor_yellow_near_x_px)), y),
                (int(round(measurement.corridor_white_near_x_px)), y),
                (0, 255, 255), 3,
            )
            cv2.circle(
                output,
                (int(round(measurement.corridor_center_near_x_px)), y),
                8, (255, 0, 255), -1,
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
