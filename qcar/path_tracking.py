"""Continuous polyline projection for shadow path-tracking evaluation."""

import math

import numpy as np


def wrap_to_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class SmoothPolylineTracker:
    """Project onto connected route segments without node-specific logic.

    The local search window and heading-consistency cost reduce accidental
    jumps to nearby lanes at intersections. Nearby segment tangents are
    blended to avoid steering discontinuities at dense waypoint boundaries.
    This class only calculates a shadow steering command.
    """

    def __init__(
        self,
        waypoints,
        stanley_gain=0.7,
        minimum_control_speed_mps=0.15,
        search_backward_segments=2,
        search_forward_segments=24,
        heading_blend_segments=3,
        heading_cost_weight_m=0.08,
        progression_cost_weight_m=0.0005,
        maximum_steering_rad=math.pi / 6.0,
    ):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[0] != 2 or points.shape[1] < 3:
            raise ValueError("waypoints must have shape (2, N), N >= 3")
        if np.linalg.norm(points[:, 0] - points[:, -1]) < 1e-9:
            points = points[:, :-1]
        self.points = points
        self.count = points.shape[1]
        self.stanley_gain = float(stanley_gain)
        self.minimum_control_speed_mps = float(minimum_control_speed_mps)
        self.search_backward_segments = int(search_backward_segments)
        self.search_forward_segments = int(search_forward_segments)
        self.heading_blend_segments = int(heading_blend_segments)
        self.heading_cost_weight_m = float(heading_cost_weight_m)
        self.progression_cost_weight_m = float(progression_cost_weight_m)
        self.maximum_steering_rad = float(maximum_steering_rad)

    def _segment(self, index):
        start_index = int(index) % self.count
        end_index = (start_index + 1) % self.count
        start = self.points[:, start_index]
        end = self.points[:, end_index]
        vector = end - start
        length = float(np.linalg.norm(vector))
        return start_index, start, vector, length

    def _blended_tangent(self, segment_index):
        accumulator = np.zeros(2, dtype=float)
        used = 0
        for offset in range(
            -self.heading_blend_segments,
            self.heading_blend_segments + 1,
        ):
            _, _, vector, length = self._segment(segment_index + offset)
            if length <= 1e-9:
                continue
            weight = 1.0 / (1.0 + abs(offset))
            accumulator += weight * vector / length
            used += 1
        magnitude = float(np.linalg.norm(accumulator))
        if used == 0 or magnitude <= 1e-9:
            return None
        return accumulator / magnitude

    def update(self, position, heading, speed, current_segment_index):
        position = np.asarray(position, dtype=float).reshape(2)
        heading = float(heading)
        current = int(current_segment_index) % self.count
        candidates = []
        for relative in range(
            -self.search_backward_segments,
            self.search_forward_segments + 1,
        ):
            index, start, vector, length = self._segment(current + relative)
            if length <= 1e-9:
                continue
            tangent = vector / length
            progress = float(np.clip(
                np.dot(position - start, tangent) / length,
                0.0,
                1.0,
            ))
            projection = start + progress * vector
            distance = float(np.linalg.norm(position - projection))
            segment_heading = math.atan2(tangent[1], tangent[0])
            heading_difference = abs(wrap_to_pi(segment_heading - heading))
            # Strongly reject an overlapping road travelling the other way.
            if heading_difference > math.pi / 2.0:
                continue
            forward_penalty = max(0, relative) * self.progression_cost_weight_m
            cost = (
                distance
                + self.heading_cost_weight_m * (1.0 - math.cos(heading_difference))
                + forward_penalty
            )
            candidates.append((cost, relative, index, projection, tangent))

        if not candidates:
            return self.unavailable(current)
        _, relative, index, projection, segment_tangent = min(
            candidates, key=lambda item: item[0]
        )
        tangent = self._blended_tangent(index)
        if tangent is None:
            tangent = segment_tangent
        reference_heading = math.atan2(tangent[1], tangent[0])
        relative_position = position - projection
        cross_track = float(-(
            tangent[0] * relative_position[1]
            - tangent[1] * relative_position[0]
        ))
        heading_error = wrap_to_pi(reference_heading - heading)
        control_speed = max(abs(float(speed)), self.minimum_control_speed_mps)
        feedback = wrap_to_pi(
            heading_error
            + math.atan2(self.stanley_gain * cross_track, control_speed)
        )
        steering = float(np.clip(
            feedback,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        ))
        return {
            "available": True,
            "segment_index": index,
            "relative_segment_offset": relative,
            "projection_x_m": float(projection[0]),
            "projection_y_m": float(projection[1]),
            "reference_heading_rad": reference_heading,
            "cross_track_error_m": cross_track,
            "heading_error_rad": heading_error,
            "control_speed_mps": control_speed,
            "feedback_steering_rad": feedback,
            "steering_rad": steering,
            "authority": 0.0,
        }

    @staticmethod
    def unavailable(index):
        return {
            "available": False,
            "segment_index": int(index),
            "relative_segment_offset": 0,
            "projection_x_m": float("nan"),
            "projection_y_m": float("nan"),
            "reference_heading_rad": float("nan"),
            "cross_track_error_m": float("nan"),
            "heading_error_rad": float("nan"),
            "control_speed_mps": float("nan"),
            "feedback_steering_rad": float("nan"),
            "steering_rad": 0.0,
            "authority": 0.0,
        }
