"""Bounded curvature-based reference correction for vehicle off-tracking."""

import numpy as np


class FootprintReferenceCorrection:
    """Estimate and smooth the outward shift needed on a curved reference.

    The SDCS route is treated as the desired rear/body path. A controller using
    a forward tracking point requires a slightly larger turn radius so the rear
    of the vehicle does not cut inside the lane. Positive output is left of the
    path tangent; negative output is right of it.
    """

    def __init__(
        self,
        wheelbase=0.256,
        correction_gain=1.0,
        maximum_offset=0.04,
        minimum_curvature=0.08,
        time_constant=0.20,
        maximum_rate=0.12,
    ):
        self.wheelbase = float(wheelbase)
        self.correction_gain = float(correction_gain)
        self.maximum_offset = float(maximum_offset)
        self.minimum_curvature = float(minimum_curvature)
        self.time_constant = float(time_constant)
        self.maximum_rate = float(maximum_rate)
        self.target_offset = 0.0
        self.applied_offset = 0.0

    def target_for_curvature(self, curvature):
        curvature = float(curvature)
        magnitude = abs(curvature)
        if magnitude < self.minimum_curvature:
            return 0.0
        radius = 1.0 / magnitude
        # Required outward displacement of a forward point when the desired
        # rear/body path has radius ``radius``.
        outward = self.correction_gain * (
            np.sqrt(radius * radius + self.wheelbase * self.wheelbase) - radius
        )
        outward = min(float(outward), self.maximum_offset)
        # Positive curvature turns left, so outward is to the right (negative
        # signed lateral offset). Right turns use the symmetric opposite sign.
        return -float(np.sign(curvature)) * outward

    def update(self, curvature, dt):
        self.target_offset = self.target_for_curvature(curvature)
        dt = max(0.0, float(dt))
        if dt <= 0.0:
            return self.applied_offset
        alpha = dt / (self.time_constant + dt)
        filtered_target = self.applied_offset + alpha * (
            self.target_offset - self.applied_offset
        )
        maximum_change = self.maximum_rate * dt
        self.applied_offset += float(np.clip(
            filtered_target - self.applied_offset,
            -maximum_change,
            maximum_change,
        ))
        return self.applied_offset
