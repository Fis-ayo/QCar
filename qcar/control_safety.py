"""General longitudinal and localization safety helpers for QCar control."""

import numpy as np


class SafeSpeedController:
    """PI speed control that never uses reverse torque as a brake."""

    def __init__(
        self,
        kp=0.1,
        ki=1.0,
        maximum_throttle=0.15,
        maximum_brake=0.20,
        integral_limit=0.05,
        static_feedforward=0.05,
        overspeed_margin_mps=0.15,
        emergency_speed_mps=0.60,
        throttle_rise_rate_per_s=0.12,
        throttle_fall_rate_per_s=0.30,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.maximum_throttle = float(maximum_throttle)
        # Retained in the signature for compatibility with earlier callers;
        # reverse motor torque is intentionally never commanded as braking.
        self.maximum_brake = float(maximum_brake)
        self.integral_limit = float(integral_limit)
        self.static_feedforward = float(static_feedforward)
        self.overspeed_margin_mps = float(overspeed_margin_mps)
        self.emergency_speed_mps = float(emergency_speed_mps)
        self.throttle_rise_rate_per_s = float(throttle_rise_rate_per_s)
        self.throttle_fall_rate_per_s = float(throttle_fall_rate_per_s)
        self.integral_error = 0.0
        self.state = "NORMAL"
        self.unsaturated_command = 0.0
        self.command = 0.0

    def _slew(self, desired, dt):
        desired = float(np.clip(desired, 0.0, self.maximum_throttle))
        if desired >= self.command:
            limit = self.throttle_rise_rate_per_s * dt
        else:
            limit = self.throttle_fall_rate_per_s * dt
        self.command += float(np.clip(desired - self.command, -limit, limit))
        self.command = float(np.clip(
            self.command, 0.0, self.maximum_throttle
        ))
        return self.command

    def update(self, speed, target_speed, dt):
        signed_speed = float(speed)
        speed = abs(signed_speed)
        target = max(0.0, float(target_speed))
        dt = max(0.0, float(dt))

        if target <= 0.01:
            self.integral_error = 0.0
            self.command = 0.0
            if speed > 0.03:
                self.state = "COASTING_TO_STOP"
                return 0.0
            self.state = "STOPPED"
            return 0.0

        overspeed = speed - target
        if speed >= self.emergency_speed_mps:
            self.integral_error = min(0.0, self.integral_error)
            self.command = 0.0
            self.state = "EMERGENCY_OVERSPEED_COAST"
            return 0.0
        if overspeed >= self.overspeed_margin_mps:
            # Unwind rather than retaining the positive effort that produced
            # the overspeed. Ordinary overspeed coasts; it never reverses.
            self.integral_error = float(np.clip(
                self.integral_error - dt * overspeed,
                -self.integral_limit,
                self.integral_limit,
            ))
            self.state = "OVERSPEED_COAST"
            return self._slew(0.0, dt)

        error = target - speed
        candidate_integral = float(np.clip(
            self.integral_error + dt * error,
            -self.integral_limit,
            self.integral_limit,
        ))
        # The QCar needs a small positive command to overcome drivetrain and
        # rolling friction.  Keeping that effort outside the integrator lets
        # the vehicle make route progress without reintroducing windup.
        feedforward = self.static_feedforward
        candidate = (
            feedforward
            + self.kp * error
            + self.ki * candidate_integral
        )
        saturated = float(np.clip(candidate, 0.0, self.maximum_throttle))
        # Conditional integration: retain the new integral only when the
        # actuator is unsaturated or the error drives it out of saturation.
        integrating_inward = (
            candidate > self.maximum_throttle and error < 0.0
        ) or (
            candidate < 0.0 and error > 0.0
        )
        if candidate == saturated or integrating_inward:
            self.integral_error = candidate_integral
        self.unsaturated_command = (
            feedforward
            + self.kp * error
            + self.ki * self.integral_error
        )
        self.state = "NORMAL"
        return self._slew(self.unsaturated_command, dt)


class TrackingLossMonitor:
    """Latch a propulsion stop after sustained unrecoverable path error."""

    def __init__(
        self,
        cross_track_limit_m=0.35,
        heading_limit_rad=np.deg2rad(90.0),
        persistence_s=0.25,
    ):
        self.cross_track_limit_m = float(cross_track_limit_m)
        self.heading_limit_rad = float(heading_limit_rad)
        self.persistence_s = float(persistence_s)
        self.risk_duration_s = 0.0
        self.state = "TRACKING"

    def update(self, cross_track_error_m, heading_error_rad, dt):
        cross_track = abs(float(cross_track_error_m))
        heading = abs(float(heading_error_rad))
        dt = max(0.0, float(dt))
        outside_recoverable_envelope = (
            cross_track >= self.cross_track_limit_m
            or heading >= self.heading_limit_rad
        )
        if self.state != "LOST":
            self.risk_duration_s = (
                self.risk_duration_s + dt
                if outside_recoverable_envelope else 0.0
            )
            if self.risk_duration_s >= self.persistence_s:
                self.state = "LOST"
        return {
            "state": self.state,
            "speed_gain": 0.0 if self.state == "LOST" else 1.0,
            "risk_duration_s": self.risk_duration_s,
        }


class LocalizationFreshnessMonitor:
    """Fail closed when GPS/LiDAR pose corrections become stale."""

    def __init__(self, stale_after_s=0.75, recover_below_s=0.25):
        self.stale_after_s = float(stale_after_s)
        self.recover_below_s = float(recover_below_s)
        self.state = "FRESH"

    def update(self, gps_age_s):
        age = float(gps_age_s)
        if not np.isfinite(age) or age > self.stale_after_s:
            self.state = "STALE"
        elif self.state == "STALE" and age <= self.recover_below_s:
            self.state = "FRESH"
        return {
            "state": self.state,
            "speed_gain": 0.0 if self.state == "STALE" else 1.0,
            "gps_age_s": age,
        }
