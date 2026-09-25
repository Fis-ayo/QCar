"""Bounded curvature-aware additions to the baseline Stanley controller."""

import numpy as np


class CurvatureAwareControl:
    """Combine filtered curvature feedforward with bounded Stanley feedback."""

    def __init__(
        self,
        analyzer,
        maximum_steering_angle=np.pi / 6,
        feedforward_gain=0.0,
        heading_gain_moderate=1.0,
        heading_gain_tight=0.55,
        heading_gain_schedule_start=0.20,
        cross_track_gain=0.7,
        minimum_feedback_speed=0.10,
        feedforward_time_constant=0.15,
        maximum_feedforward_rate=1.5,
        minimum_curve_speed=0.18,
        curvature_full_scale=1.6,
    ):
        self.analyzer = analyzer
        self.maximum_steering_angle = float(maximum_steering_angle)
        self.feedforward_gain = float(feedforward_gain)
        self.heading_gain_moderate = float(heading_gain_moderate)
        self.heading_gain_tight = float(heading_gain_tight)
        self.heading_gain_schedule_start = float(
            heading_gain_schedule_start
        )
        self.cross_track_gain = float(cross_track_gain)
        self.minimum_feedback_speed = float(minimum_feedback_speed)
        self.feedforward_time_constant = float(feedforward_time_constant)
        self.maximum_feedforward_rate = float(maximum_feedforward_rate)
        self.minimum_curve_speed = float(minimum_curve_speed)
        self.curvature_full_scale = float(curvature_full_scale)
        self.filtered_feedforward = 0.0
        self.feedback_steering = 0.0
        self.active_heading_gain = self.heading_gain_moderate
        self.curve_speed_target = float("nan")

    def update(self, waypoint_index, heading_error, cross_track_error, speed, dt):
        metrics = self.analyzer.metrics(waypoint_index)
        target_feedforward = (
            self.feedforward_gain * metrics["preview_required_steering"]
        )
        dt = max(0.0, float(dt))
        alpha = dt / (self.feedforward_time_constant + dt) if dt else 0.0
        filtered_target = self.filtered_feedforward + alpha * (
            target_feedforward - self.filtered_feedforward
        )
        maximum_change = self.maximum_feedforward_rate * dt
        self.filtered_feedforward += np.clip(
            filtered_target - self.filtered_feedforward,
            -maximum_change,
            maximum_change,
        )

        feedback_speed = max(abs(float(speed)), self.minimum_feedback_speed)
        preview_curvature = abs(metrics["preview_curvature"])
        heading_schedule = np.clip(
            (preview_curvature - self.heading_gain_schedule_start)
            / (self.curvature_full_scale - self.heading_gain_schedule_start),
            0.0,
            1.0,
        )
        self.active_heading_gain = (
            self.heading_gain_moderate
            + heading_schedule
            * (self.heading_gain_tight - self.heading_gain_moderate)
        )
        self.feedback_steering = (
            self.active_heading_gain * float(heading_error)
            + np.arctan2(
                self.cross_track_gain * float(cross_track_error),
                feedback_speed,
            )
        )
        steering = self.filtered_feedforward + self.feedback_steering
        return float(np.clip(
            steering,
            -self.maximum_steering_angle,
            self.maximum_steering_angle,
        ))

    def recommended_speed(self, nominal_speed, waypoint_index):
        metrics = self.analyzer.metrics(waypoint_index)
        ratio = min(
            1.0,
            abs(metrics["upcoming_curvature"]) / self.curvature_full_scale,
        )
        nominal_speed = float(nominal_speed)
        self.curve_speed_target = (
            nominal_speed
            - ratio * (nominal_speed - self.minimum_curve_speed)
        )
        return self.curve_speed_target
