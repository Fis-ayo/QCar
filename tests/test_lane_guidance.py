"""Tests for the zero-authority hybrid lane-guidance foundation."""

import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_guidance import (  # noqa: E402
    BoundedHybridSteering,
    BoundarySteeringConstraint,
    ContinuousCorridorAdvisor,
    FusedLaneShadowAdvisor,
    HybridLaneAdvisor,
    LaneCalibrationModel,
    LaneGuidanceReceiver,
)


class BoundedHybridSteeringTests(unittest.TestCase):
    def test_boundary_assist_adds_away_correction_without_replacing_route(self):
        fusion = BoundedHybridSteering(
            boundary_side="left",
            maximum_slew_rad_s=1.0,
        )
        result = fusion.update(0.20, {
            "state": "UNAVAILABLE",
            "confidence": 0.9,
            "boundary_category": "BEYOND_TOWARD_YELLOW_LIMIT",
        }, 0.1)
        self.assertAlmostEqual(result["constrained_waypoint_rad"], 0.20)
        self.assertAlmostEqual(result["protective_correction_rad"], -0.02)
        self.assertAlmostEqual(result["final_steering_rad"], 0.18)

    def test_calibrated_correction_is_added_once(self):
        fusion = BoundedHybridSteering(maximum_slew_rad_s=1.0)
        result = fusion.update(0.10, {
            "state": HybridLaneAdvisor.ACTIVE,
            "confidence": 0.9,
            "boundary_category": "CALIBRATED",
            "applied_correction_rad": -0.012,
        }, 0.1)
        self.assertAlmostEqual(result["protective_correction_rad"], -0.012)
        self.assertAlmostEqual(result["final_steering_rad"], 0.088)

    def test_unreliable_measurement_decays_correction_with_slew_limit(self):
        fusion = BoundedHybridSteering(maximum_slew_rad_s=0.05)
        active = fusion.update(0.10, {
            "state": "UNAVAILABLE",
            "confidence": 0.9,
            "boundary_category": "BEYOND_TOWARD_YELLOW_LIMIT",
        }, 0.2)
        missing = fusion.update(0.10, {
            "state": "UNAVAILABLE",
            "confidence": 0.1,
            "boundary_category": "FIXED_ROW_UNAVAILABLE",
        }, 0.1)
        self.assertAlmostEqual(active["protective_correction_rad"], -0.01)
        self.assertAlmostEqual(missing["protective_correction_rad"], -0.005)

    def test_unavailable_camera_does_not_change_waypoint_steering(self):
        fusion = BoundedHybridSteering()
        result = fusion.update(0.17, {
            "state": "UNAVAILABLE",
            "confidence": float("nan"),
            "boundary_category": "FIXED_ROW_UNAVAILABLE",
        }, 0.1)
        self.assertEqual(result["state"], "IDLE")
        self.assertAlmostEqual(result["final_steering_rad"], 0.17)


class ContinuousCorridorAdvisorTests(unittest.TestCase):
    @staticmethod
    def guidance(offset, confidence=0.8):
        return {
            "corridor_detected": True,
            "corridor_confidence": confidence,
            "corridor_outer_source": "dark_road_edge",
            "corridor_offset_normalized": offset,
        }

    def test_continuously_corrects_both_sides_toward_center(self):
        right = ContinuousCorridorAdvisor(maximum_slew_rad_s=10.0)
        left = ContinuousCorridorAdvisor(maximum_slew_rad_s=10.0)
        toward_left = right.update(self.guidance(0.30), 0.0, 0.0, 0.1)
        toward_right = left.update(self.guidance(-0.30), 0.0, 0.0, 0.1)
        self.assertLess(toward_left["filtered_candidate_rad"], 0.0)
        self.assertGreater(toward_right["filtered_candidate_rad"], 0.0)
        self.assertEqual(toward_left["authority"], 0.0)

    def test_curvature_reduces_camera_candidate(self):
        straight = ContinuousCorridorAdvisor(maximum_slew_rad_s=10.0)
        curve = ContinuousCorridorAdvisor(maximum_slew_rad_s=10.0)
        straight_result = straight.update(
            self.guidance(0.30), 0.0, 0.3, 0.1
        )
        curve_result = curve.update(
            self.guidance(0.30), 1.2, 0.3, 0.1
        )
        self.assertLess(
            abs(curve_result["bounded_candidate_rad"]),
            abs(straight_result["bounded_candidate_rad"]),
        )

    def test_opposing_correction_cannot_cancel_waypoint_turn(self):
        advisor = ContinuousCorridorAdvisor(maximum_slew_rad_s=10.0)
        result = advisor.update(self.guidance(1.0), 0.0, 0.05, 0.1)
        self.assertAlmostEqual(result["bounded_candidate_rad"], -0.01)
        self.assertGreater(result["hypothetical_combined_rad"], 0.0)

    def test_missing_outer_corridor_fades_to_zero(self):
        advisor = ContinuousCorridorAdvisor(maximum_slew_rad_s=0.1)
        advisor.update(self.guidance(0.5), 0.0, 0.1, 0.1)
        result = advisor.update({"corridor_detected": False}, 0.0, 0.1, 0.1)
        self.assertEqual(result["state"], "UNAVAILABLE")
        self.assertEqual(result["authority"], 0.0)

    def test_stopped_vehicle_is_unavailable(self):
        advisor = ContinuousCorridorAdvisor(maximum_slew_rad_s=10.0)
        result = advisor.update(
            self.guidance(0.5), 0.0, 0.1, 0.1, motion_allowed=False
        )
        self.assertEqual(result["state"], "UNAVAILABLE")
        self.assertEqual(result["filtered_candidate_rad"], 0.0)


class FusedLaneShadowAdvisorTests(unittest.TestCase):
    @staticmethod
    def guidance(trend="YELLOW_APPROACH", quanser=-0.5):
        return {
            "corridor_trend_state": trend,
            "corridor_detected": True,
            "corridor_confidence": 0.8,
            "corridor_outer_source": "dark_road_edge",
            "quanser_yellow_detected": True,
            "quanser_yellow_confidence": 0.5,
            "quanser_shadow_steering_rad": quanser,
        }

    def test_agreed_yellow_approach_proposes_small_shadow_correction(self):
        result = FusedLaneShadowAdvisor().update(
            self.guidance(), waypoint_steering_rad=0.20
        )
        self.assertEqual(result["state"], "SHADOW_PROPOSAL")
        self.assertAlmostEqual(result["candidate_correction_rad"], -0.01)
        self.assertAlmostEqual(result["combined_steering_rad"], 0.19)
        self.assertEqual(result["authority"], 0.0)
        self.assertEqual(result["applied_correction_rad"], 0.0)

    def test_outward_risk_blocks_yellow_correction(self):
        result = FusedLaneShadowAdvisor().update(
            self.guidance(trend="OUTWARD_APPROACH"), 0.20
        )
        self.assertEqual(result["state"], "BLOCKED_OUTWARD_RISK")
        self.assertEqual(result["candidate_correction_rad"], 0.0)

    def test_conflicting_quanser_direction_is_blocked(self):
        result = FusedLaneShadowAdvisor().update(
            self.guidance(quanser=0.2), 0.20
        )
        self.assertEqual(result["state"], "BLOCKED_CONFLICT")
        self.assertEqual(result["candidate_correction_rad"], 0.0)

    def test_stopped_vehicle_has_no_proposal(self):
        result = FusedLaneShadowAdvisor().update(
            self.guidance(), 0.20, motion_allowed=False
        )
        self.assertEqual(result["state"], "INACTIVE_STOPPED")
        self.assertEqual(result["candidate_correction_rad"], 0.0)
        self.assertFalse(result["signals_agree"])

    def test_hypothetical_combined_angle_is_clipped(self):
        result = FusedLaneShadowAdvisor().update(
            self.guidance(), -0.5235987756
        )
        self.assertAlmostEqual(
            result["combined_steering_rad"], -0.5235987756
        )

    def test_explicit_fused_authority_applies_only_agreed_candidate(self):
        advisor = FusedLaneShadowAdvisor(allow_actuation=True)
        agreed = advisor.update(self.guidance(), 0.20)
        outward = advisor.update(
            self.guidance(trend="OUTWARD_APPROACH"), 0.20
        )
        self.assertEqual(agreed["authority"], 1.0)
        self.assertAlmostEqual(agreed["applied_correction_rad"], -0.01)
        self.assertEqual(outward["authority"], 0.0)
        self.assertEqual(outward["applied_correction_rad"], 0.0)


class HybridLaneAdvisorTests(unittest.TestCase):
    @staticmethod
    def calibration_model():
        return LaneCalibrationModel({
            "calibration_id": "cal-1",
            "car_id": "car-a",
            "desired_center_to_yellow_mm": 127.0,
            "boundary_side": "left",
            "profiles": [{
                "name": "straight",
                "curvature_min_inv_m": -0.1,
                "curvature_max_inv_m": 0.1,
                "points": [
                    {"reference_x_px": 100.0, "center_to_yellow_mm": 150.0},
                    {"reference_x_px": 200.0, "center_to_yellow_mm": 100.0},
                ],
            }],
        })

    def reliable_observation(self, calibrated=False):
        return {
            "detected": True,
            "confidence": 0.90,
            "center_offset_normalized": 0.50,
            "heading_error_rad": 0.20,
            "perspective_calibrated": calibrated,
            "calibration_id": "test" if calibrated else "none",
            "age_s": 0.02,
        }

    def test_shadow_mode_never_applies_candidate(self):
        advisor = HybridLaneAdvisor(
            maximum_slew_rad_s=10.0,
            allow_actuation=False,
        )
        result = advisor.update(self.reliable_observation(calibrated=True), 0.1)
        self.assertEqual(result["state"], HybridLaneAdvisor.SHADOW_READY)
        self.assertNotEqual(result["filtered_candidate_rad"], 0.0)
        self.assertEqual(result["authority"], 0.0)
        self.assertEqual(result["applied_correction_rad"], 0.0)


class BoundarySteeringConstraintTests(unittest.TestCase):
    def test_threatened_left_boundary_limits_waypoint_and_steers_away(self):
        constraint = BoundarySteeringConstraint(boundary_side="left")
        result = constraint.update(0.230, {
            "boundary_category": "BEYOND_TOWARD_YELLOW_LIMIT",
            "applied_correction_rad": 0.0,
            "physical_lateral_error_m": float("nan"),
        })
        self.assertEqual(result["state"], "RECOVERY")
        self.assertAlmostEqual(result["constrained_waypoint_rad"], 0.0)
        self.assertAlmostEqual(result["protective_correction_rad"], -0.02)
        self.assertAlmostEqual(result["final_steering_rad"], -0.02)

    def test_recovery_holds_through_uncertain_measurement(self):
        constraint = BoundarySteeringConstraint(boundary_side="left")
        constraint.update(0.2, {
            "boundary_category": "BEYOND_TOWARD_YELLOW_LIMIT",
            "applied_correction_rad": 0.0,
        })
        result = constraint.update(0.2, {
            "boundary_category": "FIXED_ROW_UNAVAILABLE",
            "applied_correction_rad": 0.0,
        })
        self.assertEqual(result["state"], "RECOVERY")
        self.assertAlmostEqual(result["final_steering_rad"], -0.02)

    def test_calibrated_clearance_releases_constraint(self):
        constraint = BoundarySteeringConstraint(boundary_side="left")
        constraint.update(0.2, {
            "boundary_category": "BEYOND_TOWARD_YELLOW_LIMIT",
            "applied_correction_rad": 0.0,
        })
        result = constraint.update(0.1, {
            "boundary_category": "CALIBRATED",
            "applied_correction_rad": 0.01,
            "physical_lateral_error_m": -0.012,
        })
        self.assertEqual(result["state"], "NORMAL")
        self.assertAlmostEqual(result["final_steering_rad"], 0.11)


class HybridLaneAdvisorTests(HybridLaneAdvisorTests):
    def test_active_mode_requires_physical_calibration(self):
        advisor = HybridLaneAdvisor(
            maximum_slew_rad_s=10.0,
            allow_actuation=True,
        )
        uncalibrated = advisor.update(
            self.reliable_observation(calibrated=False),
            0.1,
        )
        calibrated = advisor.update(
            self.reliable_observation(calibrated=True),
            0.1,
        )
        self.assertEqual(uncalibrated["authority"], 0.0)
        self.assertEqual(calibrated["state"], HybridLaneAdvisor.ACTIVE)
        self.assertEqual(calibrated["authority"], 1.0)

    def test_unreliable_measurement_fades_candidate_to_zero(self):
        advisor = HybridLaneAdvisor(
            maximum_slew_rad_s=0.10,
            allow_actuation=False,
        )
        first = advisor.update(self.reliable_observation(), 0.5)
        missing = advisor.update({"detected": False}, 0.1)
        self.assertGreater(first["filtered_candidate_rad"], 0.0)
        self.assertLess(
            missing["filtered_candidate_rad"],
            first["filtered_candidate_rad"],
        )
        self.assertEqual(missing["state"], HybridLaneAdvisor.UNAVAILABLE)
        self.assertEqual(missing["applied_correction_rad"], 0.0)

    def test_packet_contract_rejects_incomplete_data(self):
        valid = json.dumps({
            "version": 1,
            "detected": True,
            "confidence": 0.8,
        }).encode("utf-8")
        self.assertTrue(LaneGuidanceReceiver.decode(valid)["detected"])
        with self.assertRaises(ValueError):
            LaneGuidanceReceiver.decode(
                json.dumps({"version": 1, "detected": True}).encode("utf-8")
            )

    def test_calibrated_shadow_estimates_physical_error_without_authority(self):
        advisor = HybridLaneAdvisor(
            calibration_model=self.calibration_model(),
            allow_actuation=True,
            maximum_slew_rad_s=10.0,
        )
        result = advisor.update({
            "car_id": "car-a",
            "yellow_reference_detected": True,
            "yellow_reference_confidence": 0.9,
            "yellow_reference_x_px": 150.0,
            "path_curvature_inv_m": 0.0,
            "age_s": 0.01,
        }, 0.1)

        self.assertEqual(result["state"], HybridLaneAdvisor.SHADOW_READY)
        self.assertEqual(result["calibration_profile"], "straight")
        self.assertAlmostEqual(result["estimated_center_to_yellow_mm"], 125.0)
        self.assertAlmostEqual(result["physical_lateral_error_m"], 0.002)
        self.assertEqual(result["authority"], 0.0)
        self.assertEqual(result["applied_correction_rad"], 0.0)

    def test_explicit_calibrated_authority_is_bounded(self):
        advisor = HybridLaneAdvisor(
            calibration_model=self.calibration_model(),
            allow_actuation=True,
            allow_calibrated_actuation=True,
            maximum_correction_rad=0.02,
            maximum_slew_rad_s=10.0,
        )
        result = advisor.update({
            "car_id": "car-a",
            "yellow_reference_detected": True,
            "yellow_reference_confidence": 0.9,
            "yellow_reference_x_px": 200.0,
            "path_curvature_inv_m": 0.0,
        }, 0.1)

        self.assertEqual(result["state"], HybridLaneAdvisor.ACTIVE)
        self.assertEqual(result["authority"], 1.0)
        self.assertAlmostEqual(result["applied_correction_rad"], -0.02)

    def test_calibrated_shadow_rejects_unsupported_curvature(self):
        advisor = HybridLaneAdvisor(
            calibration_model=self.calibration_model(),
            allow_actuation=False,
        )
        result = advisor.update({
            "car_id": "car-a",
            "yellow_reference_detected": True,
            "yellow_reference_confidence": 0.9,
            "yellow_reference_x_px": 150.0,
            "path_curvature_inv_m": 0.5,
        }, 0.1)

        self.assertEqual(result["state"], HybridLaneAdvisor.UNAVAILABLE)
        self.assertEqual(
            result["calibration_reason"], "UNSUPPORTED_CURVATURE"
        )
        self.assertEqual(result["applied_correction_rad"], 0.0)

    def test_calibrated_shadow_labels_toward_boundary_without_authority(self):
        advisor = HybridLaneAdvisor(
            calibration_model=self.calibration_model(),
            allow_actuation=True,
        )
        result = advisor.update({
            "car_id": "car-a",
            "yellow_reference_detected": True,
            "yellow_reference_confidence": 0.9,
            "yellow_reference_x_px": 250.0,
            "yellow_multirow_detected": [True, True, True],
            "yellow_multirow_x_px": [140.0, 190.0, 250.0],
            "path_curvature_inv_m": 0.0,
        }, 0.1)

        self.assertEqual(result["state"], HybridLaneAdvisor.UNAVAILABLE)
        self.assertEqual(result["boundary_category"], "BEYOND_TOWARD_YELLOW_LIMIT")
        self.assertEqual(result["yellow_multirow_detected_count"], 3)
        self.assertEqual(result["authority"], 0.0)
        self.assertEqual(result["applied_correction_rad"], 0.0)


if __name__ == "__main__":
    unittest.main()
