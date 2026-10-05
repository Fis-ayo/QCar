"""Shadow-mode camera guidance for future waypoint/camera steering fusion.

The classes in this module deliberately separate measurement transport from
steering authority.  A camera-derived correction can be calculated and logged,
but ``allow_actuation=False`` guarantees that the applied correction is zero.
"""

import json
import math
import socket
import time

import numpy as np


LANE_GUIDANCE_PORT = 18761


class ContinuousCorridorAdvisor:
    """Continuous two-boundary lane-centering proposal for shadow replay.

    The normalized corridor center is controlled toward the camera center.
    Curvature reduces camera authority, and the correction is constrained so
    it cannot cancel the waypoint controller's required turn.  This class has
    no node or route knowledge and never actuates steering.
    """

    def __init__(
        self,
        proportional_gain=0.06,
        derivative_gain=0.004,
        maximum_correction_rad=0.02,
        maximum_slew_rad_s=0.04,
        minimum_confidence=0.35,
        curvature_scale=0.75,
        maximum_opposing_fraction=0.20,
        offset_filter_alpha=0.20,
    ):
        self.proportional_gain = float(proportional_gain)
        self.derivative_gain = float(derivative_gain)
        self.maximum_correction_rad = float(maximum_correction_rad)
        self.maximum_slew_rad_s = float(maximum_slew_rad_s)
        self.minimum_confidence = float(minimum_confidence)
        self.curvature_scale = float(curvature_scale)
        self.maximum_opposing_fraction = float(maximum_opposing_fraction)
        self.offset_filter_alpha = float(offset_filter_alpha)
        self.filtered_offset = None
        self.previous_filtered_offset = None
        self.filtered_candidate = 0.0

    def update(
        self, guidance, path_curvature, waypoint_steering, dt,
        motion_allowed=True,
    ):
        dt = max(0.0, float(dt))
        curvature = _finite_float(path_curvature)
        waypoint = float(waypoint_steering)
        offset = _finite_float(guidance.get("corridor_offset_normalized"))
        confidence = _finite_float(guidance.get("corridor_confidence"))
        reliable = bool(
            motion_allowed
            and guidance.get("corridor_detected", False)
            and offset is not None
            and confidence is not None
            and confidence >= self.minimum_confidence
            and str(guidance.get("corridor_outer_source", "none"))
            == "dark_road_edge"
            and curvature is not None
        )

        offset_rate = float("nan")
        raw_candidate = float("nan")
        curvature_gain = 0.0
        bounded_candidate = 0.0
        reason = "CORRIDOR_UNAVAILABLE"
        if reliable:
            if self.filtered_offset is None:
                self.filtered_offset = offset
            else:
                self.filtered_offset += self.offset_filter_alpha * (
                    offset - self.filtered_offset
                )
            if self.previous_filtered_offset is None or dt <= 1e-6:
                offset_rate = 0.0
            else:
                offset_rate = float(np.clip(
                    (self.filtered_offset - self.previous_filtered_offset) / dt,
                    -2.0,
                    2.0,
                ))
            self.previous_filtered_offset = self.filtered_offset

            curvature_gain = 1.0 / (
                1.0 + self.curvature_scale * abs(curvature)
            )
            raw_candidate = curvature_gain * (
                -self.proportional_gain * self.filtered_offset
                - self.derivative_gain * offset_rate
            )
            bounded_candidate = float(np.clip(
                raw_candidate,
                -self.maximum_correction_rad,
                self.maximum_correction_rad,
            ))

            # When the camera correction opposes a route turn, it may trim
            # only a small fraction of that waypoint command. It can never
            # flatten or reverse the turn as the former recovery latch did.
            if waypoint * bounded_candidate < 0.0:
                opposing_limit = self.maximum_opposing_fraction * abs(waypoint)
                bounded_candidate = float(np.clip(
                    bounded_candidate,
                    -opposing_limit,
                    opposing_limit,
                ))
            reason = "CONTINUOUS_CORRIDOR"
        else:
            self.previous_filtered_offset = self.filtered_offset

        maximum_step = self.maximum_slew_rad_s * dt
        self.filtered_candidate += float(np.clip(
            bounded_candidate - self.filtered_candidate,
            -maximum_step,
            maximum_step,
        ))
        state = "SHADOW_READY" if reliable else "UNAVAILABLE"
        return {
            "state": state,
            "reason": reason,
            "reliable": reliable,
            "measured_offset_normalized": (
                offset if offset is not None else float("nan")
            ),
            "filtered_offset_normalized": (
                self.filtered_offset
                if self.filtered_offset is not None else float("nan")
            ),
            "offset_rate_per_s": offset_rate,
            "curvature_gain": curvature_gain,
            "raw_candidate_rad": raw_candidate,
            "bounded_candidate_rad": bounded_candidate,
            "filtered_candidate_rad": self.filtered_candidate,
            "hypothetical_combined_rad": waypoint + self.filtered_candidate,
            "authority": 0.0,
            "applied_correction_rad": 0.0,
        }


class FusedLaneShadowAdvisor:
    """Gate a tiny lane correction using independent camera evidence.

    This layer is deliberately measurement-only.  It never returns actuator
    authority; it records what a future controller would propose when the
    yellow-line direction and the paired outer-road corridor agree.
    """

    def __init__(
        self,
        boundary_side="left",
        maximum_correction_rad=0.01,
        maximum_steering_rad=0.5235987756,
        minimum_quanser_confidence=0.20,
        minimum_corridor_confidence=0.35,
        allow_actuation=False,
    ):
        if boundary_side not in {"left", "right"}:
            raise ValueError("boundary_side must be left or right")
        self.toward_sign = 1.0 if boundary_side == "left" else -1.0
        self.maximum_correction_rad = float(maximum_correction_rad)
        self.maximum_steering_rad = float(maximum_steering_rad)
        self.minimum_quanser_confidence = float(
            minimum_quanser_confidence
        )
        self.minimum_corridor_confidence = float(
            minimum_corridor_confidence
        )
        self.allow_actuation = bool(allow_actuation)

    def update(self, guidance, waypoint_steering_rad, motion_allowed=True):
        waypoint = float(waypoint_steering_rad)
        trend = str(guidance.get("corridor_trend_state", "UNAVAILABLE"))
        corridor_confidence = _finite_float(
            guidance.get("corridor_confidence")
        )
        quanser_confidence = _finite_float(
            guidance.get("quanser_yellow_confidence")
        )
        quanser_candidate = _finite_float(
            guidance.get("quanser_shadow_steering_rad")
        )
        corridor_ready = (
            bool(guidance.get("corridor_detected", False))
            and corridor_confidence is not None
            and corridor_confidence >= self.minimum_corridor_confidence
            and str(guidance.get("corridor_outer_source", "none"))
            == "dark_road_edge"
        )
        quanser_ready = (
            bool(guidance.get("quanser_yellow_detected", False))
            and quanser_confidence is not None
            and quanser_confidence >= self.minimum_quanser_confidence
            and quanser_candidate is not None
        )
        quanser_points_away = bool(
            quanser_ready and quanser_candidate * self.toward_sign < 0.0
        )

        candidate = 0.0
        if not motion_allowed:
            state = "INACTIVE_STOPPED"
            reason = "MOTION_NOT_ALLOWED"
        elif trend in {"OUTWARD_APPROACH", "OUTWARD_LOST"}:
            state = "BLOCKED_OUTWARD_RISK"
            reason = trend
        elif not corridor_ready:
            state = "NO_PROPOSAL"
            reason = "CORRIDOR_UNAVAILABLE"
        elif not quanser_ready:
            state = "NO_PROPOSAL"
            reason = "QUANSER_UNAVAILABLE"
        elif trend != "YELLOW_APPROACH":
            state = "MONITORING"
            reason = trend
        elif not quanser_points_away:
            state = "BLOCKED_CONFLICT"
            reason = "QUANSER_NOT_AWAY_FROM_YELLOW"
        else:
            state = "SHADOW_PROPOSAL"
            reason = "YELLOW_AND_CORRIDOR_AGREE"
            candidate = float(np.clip(
                quanser_candidate,
                -self.maximum_correction_rad,
                self.maximum_correction_rad,
            ))

        authority = 1.0 if (
            self.allow_actuation and state == "SHADOW_PROPOSAL"
        ) else 0.0
        applied = authority * candidate
        return {
            "state": state,
            "reason": reason,
            "candidate_correction_rad": candidate,
            "combined_steering_rad": float(np.clip(
                waypoint + candidate,
                -self.maximum_steering_rad,
                self.maximum_steering_rad,
            )),
            "authority": authority,
            "applied_correction_rad": applied,
            "corridor_ready": corridor_ready,
            "quanser_ready": quanser_ready,
            "signals_agree": bool(
                motion_allowed
                and trend == "YELLOW_APPROACH"
                and corridor_ready
                and quanser_points_away
            ),
        }


class BoundedHybridSteering:
    """Add one small camera correction without replacing waypoint steering."""

    def __init__(
        self,
        boundary_side="left",
        maximum_correction_rad=0.02,
        maximum_slew_rad_s=0.05,
        minimum_confidence=0.45,
        maximum_steering_rad=0.5235987756,
    ):
        if boundary_side not in {"left", "right"}:
            raise ValueError("boundary_side must be left or right")
        self.toward_sign = 1.0 if boundary_side == "left" else -1.0
        self.maximum_correction_rad = float(maximum_correction_rad)
        self.maximum_slew_rad_s = float(maximum_slew_rad_s)
        self.minimum_confidence = float(minimum_confidence)
        self.maximum_steering_rad = float(maximum_steering_rad)
        self.correction_rad = 0.0

    def update(self, waypoint_steering_rad, guidance, dt):
        waypoint = float(waypoint_steering_rad)
        confidence = _finite_float(guidance.get("confidence"))
        category = str(guidance.get("boundary_category", "UNAVAILABLE"))
        state = str(guidance.get("state", "UNAVAILABLE"))
        applied = _finite_float(guidance.get("applied_correction_rad"))
        reliable = confidence is not None and confidence >= self.minimum_confidence

        target = 0.0
        reason = "CAMERA_UNAVAILABLE"
        if reliable and category == "BEYOND_TOWARD_YELLOW_LIMIT":
            # Preserve the route turn and add only a small correction away
            # from the known divider. Unlike the former recovery constraint,
            # this never zeros or latches waypoint steering.
            target = -self.toward_sign * self.maximum_correction_rad
            reason = "BOUNDARY_AWAY_ASSIST"
        elif reliable and state == HybridLaneAdvisor.ACTIVE and applied is not None:
            target = float(np.clip(
                applied,
                -self.maximum_correction_rad,
                self.maximum_correction_rad,
            ))
            reason = "CALIBRATED_CENTERING"

        dt = max(0.0, float(dt))
        maximum_step = self.maximum_slew_rad_s * dt
        self.correction_rad += float(np.clip(
            target - self.correction_rad,
            -maximum_step,
            maximum_step,
        ))
        final = float(np.clip(
            waypoint + self.correction_rad,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        ))
        return {
            "state": "ACTIVE" if abs(self.correction_rad) > 1e-9 else "IDLE",
            "reason": reason,
            "unconstrained_combined_rad": waypoint + self.correction_rad,
            "constrained_waypoint_rad": waypoint,
            "protective_correction_rad": self.correction_rad,
            "constraint_adjustment_rad": 0.0,
            "final_steering_rad": final,
        }


class BoundarySteeringConstraint:
    """Bound waypoint steering while recovering from a threatened boundary.

    This is a low-speed safety supervisor, not a replacement path tracker.
    For a left yellow boundary, positive steering is toward the boundary and
    negative steering is away.  Recovery remains latched until calibrated
    clearance is restored or the boundary is explicitly beyond the away side.
    """

    NORMAL = "NORMAL"
    RECOVERY = "RECOVERY"

    def __init__(
        self,
        boundary_side="left",
        maximum_toward_waypoint_rad=0.0,
        protective_correction_rad=0.02,
        recovery_margin_m=0.010,
        maximum_steering_rad=0.5235987756,
    ):
        if boundary_side not in {"left", "right"}:
            raise ValueError("boundary_side must be left or right")
        self.boundary_side = boundary_side
        self.toward_sign = 1.0 if boundary_side == "left" else -1.0
        self.maximum_toward_waypoint_rad = float(maximum_toward_waypoint_rad)
        self.protective_correction_rad = float(protective_correction_rad)
        self.recovery_margin_m = float(recovery_margin_m)
        self.maximum_steering_rad = float(maximum_steering_rad)
        self.state = self.NORMAL
        self.last_protective_correction_rad = 0.0

    def update(self, waypoint_steering_rad, guidance):
        waypoint = float(waypoint_steering_rad)
        category = str(guidance.get("boundary_category", "UNAVAILABLE"))
        applied = float(guidance.get("applied_correction_rad", 0.0))
        physical_error = _finite_float(
            guidance.get("physical_lateral_error_m")
        )

        # Remember only a correction that points away from the known boundary.
        if applied * self.toward_sign < 0.0:
            self.last_protective_correction_rad = applied

        if category == "BEYOND_TOWARD_YELLOW_LIMIT":
            self.state = self.RECOVERY

        recovered = (
            category == "BEYOND_AWAY_LIMIT"
            or (
                category == "CALIBRATED"
                and physical_error is not None
                and physical_error <= -self.recovery_margin_m
            )
        )
        if self.state == self.RECOVERY and recovered:
            self.state = self.NORMAL

        unconstrained = waypoint + applied
        constrained_waypoint = waypoint
        protective = applied
        if self.state == self.RECOVERY:
            signed_waypoint = self.toward_sign * waypoint
            if signed_waypoint > self.maximum_toward_waypoint_rad:
                constrained_waypoint = self.toward_sign * (
                    self.maximum_toward_waypoint_rad
                )
            minimum_away = -self.toward_sign * self.protective_correction_rad
            remembered = self.last_protective_correction_rad
            if remembered * self.toward_sign >= 0.0:
                remembered = minimum_away
            protective = (
                min(remembered, minimum_away)
                if self.toward_sign > 0.0
                else max(remembered, minimum_away)
            )

        final = float(np.clip(
            constrained_waypoint + protective,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        ))
        return {
            "state": self.state,
            "unconstrained_combined_rad": unconstrained,
            "constrained_waypoint_rad": constrained_waypoint,
            "protective_correction_rad": protective,
            "constraint_adjustment_rad": final - unconstrained,
            "final_steering_rad": final,
        }


class LaneCalibrationModel:
    """Convert fixed-row yellow position to physical lateral displacement."""

    def __init__(self, configuration):
        self.configuration = dict(configuration)
        self.calibration_id = str(configuration["calibration_id"])
        self.car_id = str(configuration["car_id"])
        self.desired_distance_mm = float(
            configuration["desired_center_to_yellow_mm"]
        )
        self.boundary_side = str(configuration["boundary_side"])
        self.profiles = []
        for source in configuration["profiles"]:
            profile = dict(source)
            points = sorted(
                (
                    float(point["reference_x_px"]),
                    float(point["center_to_yellow_mm"]),
                )
                for point in profile["points"]
            )
            if len(points) < 2 or len({point[0] for point in points}) != len(points):
                raise ValueError("Calibration profiles require unique pixel points")
            profile["points"] = points
            profile["curvature_min_inv_m"] = float(
                profile["curvature_min_inv_m"]
            )
            profile["curvature_max_inv_m"] = float(
                profile["curvature_max_inv_m"]
            )
            self.profiles.append(profile)

    @classmethod
    def from_file(cls, path):
        with open(path, "r", encoding="utf-8") as handle:
            return cls(json.load(handle))

    def estimate(self, reference_x_px, path_curvature_inv_m, car_id):
        reference_x_px = _finite_float(reference_x_px)
        curvature = _finite_float(path_curvature_inv_m)
        if reference_x_px is None or curvature is None:
            return self.unavailable("NONFINITE_INPUT")
        if str(car_id) != self.car_id:
            return self.unavailable("CAR_ID_MISMATCH")
        matches = [
            profile for profile in self.profiles
            if profile["curvature_min_inv_m"] <= curvature
            <= profile["curvature_max_inv_m"]
        ]
        if len(matches) != 1:
            return self.unavailable("UNSUPPORTED_CURVATURE")
        profile = matches[0]
        pixels = [point[0] for point in profile["points"]]
        distances = [point[1] for point in profile["points"]]
        if reference_x_px < pixels[0]:
            return self.unavailable("BEYOND_AWAY_LIMIT", profile["name"])
        if reference_x_px > pixels[-1]:
            return self.unavailable("BEYOND_TOWARD_YELLOW_LIMIT", profile["name"])
        estimated_distance = float(np.interp(reference_x_px, pixels, distances))
        # Positive error means the vehicle is closer to a left boundary than
        # the centered reference. This is a physical estimate, not authority.
        sign = 1.0 if self.boundary_side == "left" else -1.0
        lateral_error_m = sign * (
            self.desired_distance_mm - estimated_distance
        ) / 1000.0
        return {
            "available": True,
            "reason": "CALIBRATED",
            "profile": str(profile["name"]),
            "estimated_center_to_yellow_mm": estimated_distance,
            "lateral_error_m": lateral_error_m,
            "calibration_id": self.calibration_id,
        }

    def unavailable(self, reason, profile="none"):
        return {
            "available": False,
            "reason": str(reason),
            "profile": str(profile),
            "estimated_center_to_yellow_mm": float("nan"),
            "lateral_error_m": float("nan"),
            "calibration_id": self.calibration_id,
        }


class HybridLaneAdvisor:
    """Produce a bounded camera correction candidate for shadow evaluation."""

    UNAVAILABLE = "UNAVAILABLE"
    SHADOW_READY = "SHADOW_READY"
    ACTIVE = "ACTIVE"

    def __init__(
        self,
        minimum_confidence=0.45,
        lateral_gain=0.20,
        heading_gain=0.15,
        steering_sign=1.0,
        maximum_correction_rad=0.12,
        maximum_slew_rad_s=0.30,
        allow_actuation=False,
        allow_calibrated_actuation=False,
        calibration_model=None,
        physical_lateral_gain_rad_per_m=1.0,
    ):
        self.minimum_confidence = float(minimum_confidence)
        self.lateral_gain = float(lateral_gain)
        self.heading_gain = float(heading_gain)
        self.steering_sign = float(steering_sign)
        self.maximum_correction_rad = float(maximum_correction_rad)
        self.maximum_slew_rad_s = float(maximum_slew_rad_s)
        self.allow_actuation = bool(allow_actuation)
        self.allow_calibrated_actuation = bool(allow_calibrated_actuation)
        self.calibration_model = calibration_model
        self.physical_lateral_gain_rad_per_m = float(
            physical_lateral_gain_rad_per_m
        )
        self.filtered_candidate = 0.0

    def update(self, observation, dt):
        calibrated = None
        if self.calibration_model is not None:
            fixed_detected = bool(
                observation.get("yellow_reference_detected", False)
            )
            fixed_confidence = _finite_float(
                observation.get("yellow_reference_confidence")
            )
            if fixed_detected and fixed_confidence is not None:
                calibrated = self.calibration_model.estimate(
                    observation.get("yellow_reference_x_px"),
                    observation.get("path_curvature_inv_m"),
                    observation.get("car_id", ""),
                )
            else:
                calibrated = self.calibration_model.unavailable(
                    "FIXED_ROW_UNAVAILABLE"
                )

        detected = bool(observation.get("detected", False))
        confidence = _finite_float(observation.get("confidence"))
        lateral_offset = _finite_float(observation.get("center_offset_normalized"))
        heading_error = _finite_float(observation.get("heading_error_rad"))
        perspective_calibrated = bool(
            observation.get("perspective_calibrated", False)
        )
        legacy_reliable = (
            detected
            and confidence is not None
            and confidence >= self.minimum_confidence
            and lateral_offset is not None
            and heading_error is not None
        )
        calibrated_reliable = bool(
            calibrated is not None
            and calibrated["available"]
            and fixed_confidence is not None
            and fixed_confidence >= self.minimum_confidence
        )
        reliable = calibrated_reliable if calibrated is not None else legacy_reliable

        if reliable:
            if calibrated_reliable:
                # A positive left-boundary error means too close to the left;
                # the shadow proposal steers right (negative). Direction must
                # still be validated before any authority can be enabled.
                raw_candidate = -self.physical_lateral_gain_rad_per_m * (
                    calibrated["lateral_error_m"]
                )
            else:
                raw_candidate = self.steering_sign * (
                    self.lateral_gain * lateral_offset
                    + self.heading_gain * heading_error
                )
            bounded_candidate = float(np.clip(
                raw_candidate,
                -self.maximum_correction_rad,
                self.maximum_correction_rad,
            ))
            maximum_step = self.maximum_slew_rad_s * max(float(dt), 0.0)
            self.filtered_candidate += float(np.clip(
                bounded_candidate - self.filtered_candidate,
                -maximum_step,
                maximum_step,
            ))
        else:
            raw_candidate = float("nan")
            bounded_candidate = 0.0
            maximum_step = self.maximum_slew_rad_s * max(float(dt), 0.0)
            self.filtered_candidate += float(np.clip(
                -self.filtered_candidate,
                -maximum_step,
                maximum_step,
            ))

        # Active steering requires both explicit software authority and a
        # physically validated image-to-road calibration.
        # This calibrated estimator is intentionally shadow-only. Perspective
        # calibration alone is not permission to actuate.
        calibrated_authorized = (
            self.calibration_model is not None
            and self.allow_calibrated_actuation
            and calibrated_reliable
        )
        legacy_authorized = (
            self.calibration_model is None and perspective_calibrated
        )
        authority = 1.0 if (
            self.allow_actuation
            and reliable
            and (calibrated_authorized or legacy_authorized)
        ) else 0.0
        applied_correction = authority * self.filtered_candidate
        state = (
            self.ACTIVE if authority > 0.0
            else self.SHADOW_READY if reliable
            else self.UNAVAILABLE
        )
        return {
            "state": state,
            "reliable": reliable,
            "confidence": (
                fixed_confidence if calibrated is not None
                and fixed_confidence is not None
                else confidence if confidence is not None else float("nan")
            ),
            "center_offset_normalized": (
                lateral_offset if lateral_offset is not None else float("nan")
            ),
            "heading_error_rad": (
                heading_error if heading_error is not None else float("nan")
            ),
            "raw_candidate_rad": raw_candidate,
            "bounded_candidate_rad": bounded_candidate,
            "filtered_candidate_rad": self.filtered_candidate,
            "authority": authority,
            "applied_correction_rad": applied_correction,
            "perspective_calibrated": perspective_calibrated,
            "calibration_id": str(observation.get("calibration_id", "none")),
            "age_s": float(observation.get("age_s", float("inf"))),
            "calibration_profile": (
                calibrated["profile"] if calibrated is not None else "legacy"
            ),
            "calibration_reason": (
                calibrated["reason"] if calibrated is not None else "LEGACY"
            ),
            "estimated_center_to_yellow_mm": (
                calibrated["estimated_center_to_yellow_mm"]
                if calibrated is not None else float("nan")
            ),
            "physical_lateral_error_m": (
                calibrated["lateral_error_m"]
                if calibrated is not None else float("nan")
            ),
            "path_curvature_inv_m": float(
                observation.get("path_curvature_inv_m", float("nan"))
            ),
            "yellow_reference_x_px": float(
                observation.get("yellow_reference_x_px", float("nan"))
            ),
            "boundary_category": (
                calibrated["reason"] if calibrated is not None else "LEGACY"
            ),
            "yellow_multirow_detected_count": sum(
                bool(value) for value in observation.get(
                    "yellow_multirow_detected", []
                )
            ),
            "yellow_multirow_x_px": tuple(
                float(value) for value in observation.get(
                    "yellow_multirow_x_px", []
                )
            ),
            "corridor_detected": bool(
                observation.get("corridor_detected", False)
            ),
            "corridor_confidence": float(
                observation.get("corridor_confidence", float("nan"))
            ),
            "corridor_paired_rows": int(
                observation.get("corridor_paired_rows", 0)
            ),
            "corridor_offset_normalized": float(
                observation.get("corridor_offset_normalized", float("nan"))
            ),
            "corridor_heading_rad": float(
                observation.get("corridor_heading_rad", float("nan"))
            ),
            "corridor_outer_source": str(
                observation.get("corridor_outer_source", "none")
            ),
            "quanser_yellow_detected": bool(
                observation.get("quanser_yellow_detected", False)
            ),
            "quanser_yellow_confidence": float(
                observation.get("quanser_yellow_confidence", float("nan"))
            ),
            "quanser_yellow_slope": float(
                observation.get("quanser_yellow_slope", float("nan"))
            ),
            "quanser_yellow_intercept": float(
                observation.get("quanser_yellow_intercept", float("nan"))
            ),
            "quanser_shadow_steering_rad": float(
                observation.get("quanser_shadow_steering_rad", float("nan"))
            ),
            "corridor_trend_state": str(
                observation.get("corridor_trend_state", "UNAVAILABLE")
            ),
            "corridor_trend_offset_delta": float(
                observation.get("corridor_trend_offset_delta", float("nan"))
            ),
            "corridor_trend_heading_delta": float(
                observation.get("corridor_trend_heading_delta", float("nan"))
            ),
        }


class LaneGuidancePublisher:
    def __init__(self, host="127.0.0.1", port=LANE_GUIDANCE_PORT, car_id=""):
        self.address = (host, int(port))
        self.car_id = str(car_id)
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, observation):
        self.sequence += 1
        packet = dict(observation)
        packet.update({
            "version": 1,
            "car_id": self.car_id,
            "sequence": self.sequence,
            "sent_wall_time_s": time.time(),
        })
        self.socket.sendto(
            json.dumps(packet, separators=(",", ":")).encode("utf-8"),
            self.address,
        )

    def close(self):
        self.socket.close()


class LaneGuidanceReceiver:
    """Receive the latest camera observation without granting it authority."""

    def __init__(
        self,
        host="127.0.0.1",
        port=LANE_GUIDANCE_PORT,
        expected_car_id=None,
        stale_after_s=0.50,
    ):
        self.expected_car_id = expected_car_id
        self.stale_after_s = float(stale_after_s)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)
        self.last_packet = None
        self.last_received_monotonic = None

    @staticmethod
    def decode(data):
        packet = json.loads(data.decode("utf-8"))
        if packet.get("version") != 1:
            raise ValueError("Unsupported lane-guidance packet version")
        if "detected" not in packet or "confidence" not in packet:
            raise ValueError("Incomplete lane-guidance packet")
        return packet

    def receive(self, now=None):
        now = time.monotonic() if now is None else float(now)
        while True:
            try:
                data, _ = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                packet = self.decode(data)
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                continue
            if (
                self.expected_car_id
                and packet.get("car_id") != self.expected_car_id
            ):
                continue
            self.last_packet = packet
            self.last_received_monotonic = now

        if (
            self.last_packet is None
            or self.last_received_monotonic is None
            or now - self.last_received_monotonic > self.stale_after_s
        ):
            return unavailable_observation(
                float("inf") if self.last_received_monotonic is None
                else now - self.last_received_monotonic
            )

        result = dict(self.last_packet)
        result["age_s"] = now - self.last_received_monotonic
        return result

    def close(self):
        self.socket.close()


def unavailable_observation(age_s=float("inf")):
    return {
        "detected": False,
        "confidence": float("nan"),
        "center_offset_normalized": float("nan"),
        "heading_error_rad": float("nan"),
        "perspective_calibrated": False,
        "calibration_id": "none",
        "age_s": float(age_s),
        "quanser_yellow_detected": False,
        "quanser_yellow_confidence": float("nan"),
        "quanser_yellow_slope": float("nan"),
        "quanser_yellow_intercept": float("nan"),
        "quanser_shadow_steering_rad": float("nan"),
        "corridor_trend_state": "UNAVAILABLE",
        "corridor_trend_offset_delta": float("nan"),
        "corridor_trend_heading_delta": float("nan"),
    }


def _finite_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None
