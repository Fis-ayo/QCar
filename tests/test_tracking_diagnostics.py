"""Local tests for measurement-only route-tracking diagnostics."""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from tracking_diagnostics import (  # noqa: E402
    PathCurvatureAnalyzer,
    TrackingDiagnostics,
    VehicleFootprintEstimator,
)


class StubController:
    def __init__(self):
        self.wp = np.array(
            [[0.0, 1.0, 2.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
        )
        self.N = 4
        self.wpi = 1
        self.p_ref = np.array([1.2, 0.2])
        self.th_ref = 0.75
        self.cross_track_error = -0.04
        self.heading_error = 0.12
        self.distance_to_next_waypoint = 0.8
        self.maxSteeringAngle = np.pi / 6


class TrackingDiagnosticsTests(unittest.TestCase):
    def test_records_reference_and_rate_limits_samples(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            csv_path = Path(temporary_directory) / "tracking.csv"
            controller = StubController()
            diagnostics = TrackingDiagnostics(
                car_id="car-a",
                route_id="ROUTE_A",
                waypoints=controller.wp,
                csv_path=str(csv_path),
                sample_period=0.05,
            )
            recorded = diagnostics.record(
                elapsed=1.0,
                pose=[0.9, 0.1, 0.2],
                tracking_point=[1.1, 0.14],
                controller=controller,
                speed=0.2,
                target_speed=0.25,
                speed_gain=1.0,
                throttle=0.08,
                steering=controller.maxSteeringAngle,
                intersection_phase="CRUISING",
            )
            skipped = diagnostics.record(
                elapsed=1.01,
                pose=[0.9, 0.1, 0.2],
                tracking_point=[1.1, 0.14],
                controller=controller,
                speed=0.2,
                target_speed=0.25,
                speed_gain=1.0,
                throttle=0.08,
                steering=0.0,
                intersection_phase="CRUISING",
            )
            diagnostics.close()

            self.assertTrue(recorded)
            self.assertFalse(skipped)
            with csv_path.open(newline="", encoding="utf-8") as csv_file:
                rows = list(csv.DictReader(csv_file))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["car_id"], "car-a")
            self.assertEqual(rows[0]["route_id"], "ROUTE_A")
            self.assertEqual(rows[0]["waypoint_index"], "1")
            self.assertEqual(rows[0]["steering_saturated"], "1")
            self.assertAlmostEqual(float(rows[0]["segment_end_y_m"]), 1.0)
            self.assertEqual(
                rows[0]["geometry_assumption"],
                "ekf_pose_assumed_body_center",
            )
            self.assertEqual(rows[0]["lane_guidance_state"], "DISABLED")
            self.assertEqual(
                float(rows[0]["lane_guidance_authority"]),
                0.0,
            )
            self.assertEqual(
                float(rows[0]["lane_guidance_applied_correction_rad"]),
                0.0,
            )
            self.assertEqual(
                rows[0]["lane_guidance_calibration_profile"], "none"
            )
            self.assertEqual(
                rows[0]["lane_guidance_calibration_reason"], "DISABLED"
            )
            self.assertEqual(
                rows[0]["lane_guidance_boundary_category"], "DISABLED"
            )
            self.assertEqual(
                int(rows[0]["lane_guidance_multirow_detected_count"]), 0
            )
            self.assertEqual(rows[0]["boundary_safety_state"], "DISABLED")
            self.assertEqual(float(rows[0]["boundary_safety_gain"]), 1.0)
            self.assertEqual(rows[0]["boundary_steering_state"], "DISABLED")
            self.assertEqual(int(rows[0]["corridor_detected"]), 0)
            self.assertEqual(int(rows[0]["quanser_yellow_detected"]), 0)
            self.assertEqual(rows[0]["corridor_trend_state"], "UNAVAILABLE")
            self.assertEqual(rows[0]["fused_lane_state"], "DISABLED")
            self.assertEqual(float(rows[0]["fused_lane_authority"]), 0.0)
            self.assertEqual(
                rows[0]["continuous_corridor_state"], "DISABLED"
            )
            self.assertEqual(
                float(rows[0]["continuous_corridor_authority"]), 0.0
            )
            self.assertEqual(int(rows[0]["smooth_path_available"]), 0)
            self.assertEqual(float(rows[0]["smooth_path_authority"]), 0.0)
            self.assertEqual(rows[0]["speed_control_state"], "UNKNOWN")
            self.assertEqual(
                rows[0]["localization_safety_state"], "UNKNOWN"
            )

    def test_curvature_and_required_steering_on_circle(self):
        radius = 1.0
        angles = np.linspace(0.0, 2.0 * np.pi, 361)
        waypoints = np.vstack((radius * np.cos(angles), radius * np.sin(angles)))
        analyzer = PathCurvatureAnalyzer(
            waypoints,
            wheelbase=0.256,
            half_window_distance=0.075,
            lookahead_distance=0.5,
        )
        metrics = analyzer.metrics(90)
        self.assertAlmostEqual(metrics["local_curvature"], 1.0, places=2)
        self.assertAlmostEqual(metrics["preview_curvature"], 1.0, places=2)
        self.assertAlmostEqual(metrics["upcoming_curvature"], 1.0, places=2)
        self.assertAlmostEqual(
            metrics["local_required_steering"],
            np.arctan(0.256),
            places=2,
        )

    def test_footprint_uses_documented_axle_and_track_offsets(self):
        estimator = VehicleFootprintEstimator(
            half_wheelbase=0.130,
            half_track=0.086,
        )
        points, errors, inner_side, _ = estimator.estimate(
            pose=[1.0, 2.0, 0.0],
            segment_start=np.array([0.0, 0.0]),
            segment_end=np.array([2.0, 0.0]),
            curvature=1.0,
        )
        np.testing.assert_allclose(points["front_axle_center"], [1.13, 2.0])
        np.testing.assert_allclose(points["rear_axle_center"], [0.87, 2.0])
        np.testing.assert_allclose(points["front_left_wheel"], [1.13, 2.086])
        self.assertAlmostEqual(errors["body_center"], -2.0)
        self.assertEqual(inner_side, "left")


if __name__ == "__main__":
    unittest.main()
