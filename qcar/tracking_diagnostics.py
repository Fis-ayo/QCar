"""Measurement-only diagnostics for route and lateral tracking.

The recorder is intentionally independent of a particular route, node, or
vehicle.  It records the reference presented to the steering controller and
the resulting estimated motion, but never changes controller behavior.
"""

import csv
import time

import numpy as np


class PathCurvatureAnalyzer:
    """Precompute robust signed curvature on a dense cyclic waypoint path."""

    def __init__(
        self,
        waypoints,
        wheelbase=0.256,
        half_window_distance=0.075,
        lookahead_distance=0.50,
        preview_distance=0.20,
    ):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[0] != 2 or points.shape[1] < 4:
            raise ValueError("Waypoints must have shape (2, N) with N >= 4.")
        # SDCS cyclic paths repeat the first point at the end. Keep one copy.
        if np.linalg.norm(points[:, 0] - points[:, -1]) < 1e-6:
            points = points[:, :-1]
        self.points = points
        self.count = points.shape[1]
        self.wheelbase = float(wheelbase)
        self.half_window_distance = float(half_window_distance)
        self.lookahead_distance = float(lookahead_distance)
        self.preview_distance = float(preview_distance)
        self.curvature = np.asarray(
            [self._curvature_at(index) for index in range(self.count)],
            dtype=float,
        )

    def _offset_index(self, index, direction, distance):
        current = int(index) % self.count
        travelled = 0.0
        for _ in range(self.count - 1):
            following = (current + direction) % self.count
            travelled += float(
                np.linalg.norm(
                    self.points[:, following] - self.points[:, current]
                )
            )
            current = following
            if travelled >= distance:
                break
        return current

    def _curvature_at(self, index):
        previous = self._offset_index(
            index, -1, self.half_window_distance
        )
        following = self._offset_index(
            index, 1, self.half_window_distance
        )
        first = self.points[:, previous]
        middle = self.points[:, int(index) % self.count]
        last = self.points[:, following]
        side_a = float(np.linalg.norm(middle - first))
        side_b = float(np.linalg.norm(last - middle))
        side_c = float(np.linalg.norm(last - first))
        denominator = side_a * side_b * side_c
        if denominator <= 1e-9:
            return 0.0
        first_vector = middle - first
        second_vector = last - middle
        cross = float(
            first_vector[0] * second_vector[1]
            - first_vector[1] * second_vector[0]
        )
        return 2.0 * cross / denominator

    def metrics(self, waypoint_index):
        index = int(waypoint_index) % self.count
        local_curvature = float(self.curvature[index])
        preview_index = self._offset_index(
            index, 1, self.preview_distance
        )
        preview_curvature = float(self.curvature[preview_index])
        upcoming_indices = [index]
        current = index
        travelled = 0.0
        for _ in range(self.count - 1):
            following = (current + 1) % self.count
            travelled += float(
                np.linalg.norm(
                    self.points[:, following] - self.points[:, current]
                )
            )
            if travelled > self.lookahead_distance:
                break
            upcoming_indices.append(following)
            current = following
        upcoming_values = self.curvature[upcoming_indices]
        upcoming_curvature = float(
            upcoming_values[np.argmax(np.abs(upcoming_values))]
        )
        return {
            "local_curvature": local_curvature,
            "preview_curvature": preview_curvature,
            "upcoming_curvature": upcoming_curvature,
            "local_required_steering": float(
                np.arctan(self.wheelbase * local_curvature)
            ),
            "preview_required_steering": float(
                np.arctan(self.wheelbase * preview_curvature)
            ),
            "upcoming_required_steering": float(
                np.arctan(self.wheelbase * upcoming_curvature)
            ),
        }


class VehicleFootprintEstimator:
    """Estimate QCar axle/wheel locations from an assumed body-center pose."""

    def __init__(self, half_wheelbase=0.130, half_track=0.086):
        self.half_wheelbase = float(half_wheelbase)
        self.half_track = float(half_track)

    @staticmethod
    def _signed_error(point, segment_start, segment_end):
        segment = segment_end - segment_start
        length = float(np.linalg.norm(segment))
        if length <= 1e-9:
            return float("nan")
        tangent = segment / length
        relative = point - segment_start
        # Match the sign convention used by SteeringController: points to the
        # left of the path tangent have negative cross-track error.
        return float(-(tangent[0] * relative[1] - tangent[1] * relative[0]))

    def estimate(self, pose, segment_start, segment_end, curvature):
        pose = np.asarray(pose, dtype=float).reshape(3)
        center = pose[:2]
        forward = np.array([np.cos(pose[2]), np.sin(pose[2])])
        left = np.array([-np.sin(pose[2]), np.cos(pose[2])])
        front = center + self.half_wheelbase * forward
        rear = center - self.half_wheelbase * forward
        points = {
            "body_center": center,
            "front_axle_center": front,
            "rear_axle_center": rear,
            "front_left_wheel": front + self.half_track * left,
            "front_right_wheel": front - self.half_track * left,
            "rear_left_wheel": rear + self.half_track * left,
            "rear_right_wheel": rear - self.half_track * left,
        }
        errors = {
            name: self._signed_error(point, segment_start, segment_end)
            for name, point in points.items()
        }
        if curvature > 1e-9:
            inner_side = "left"
        elif curvature < -1e-9:
            inner_side = "right"
        else:
            inner_side = "none"
        inner_errors = []
        if inner_side != "none":
            inner_errors = [
                errors[f"front_{inner_side}_wheel"],
                errors[f"rear_{inner_side}_wheel"],
            ]
        if inner_side == "left":
            inner_wheel_error = min(inner_errors)
        elif inner_side == "right":
            inner_wheel_error = max(inner_errors)
        else:
            inner_wheel_error = float("nan")
        return points, errors, inner_side, inner_wheel_error


class TrackingDiagnostics:
    def __init__(
        self,
        car_id,
        route_id,
        waypoints,
        csv_path="tracking_diagnostics.csv",
        sample_period=0.05,
        wheelbase=0.256,
        maximum_steering_angle=np.pi / 6,
    ):
        self.car_id = car_id
        self.route_id = route_id
        self.sample_period = float(sample_period)
        self.maximum_steering_angle = float(maximum_steering_angle)
        self.curvature_analyzer = PathCurvatureAnalyzer(
            waypoints=waypoints,
            wheelbase=wheelbase,
        )
        self.footprint_estimator = VehicleFootprintEstimator()
        self.last_sample_time = float("-inf")
        self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "elapsed_s",
            "wall_time_s",
            "car_id",
            "route_id",
            "ekf_x_m",
            "ekf_y_m",
            "ekf_heading_rad",
            "tracking_x_m",
            "tracking_y_m",
            "geometry_assumption",
            "front_axle_x_m",
            "front_axle_y_m",
            "rear_axle_x_m",
            "rear_axle_y_m",
            "front_left_wheel_x_m",
            "front_left_wheel_y_m",
            "front_right_wheel_x_m",
            "front_right_wheel_y_m",
            "rear_left_wheel_x_m",
            "rear_left_wheel_y_m",
            "rear_right_wheel_x_m",
            "rear_right_wheel_y_m",
            "waypoint_index",
            "segment_start_x_m",
            "segment_start_y_m",
            "segment_end_x_m",
            "segment_end_y_m",
            "reference_x_m",
            "reference_y_m",
            "reference_heading_rad",
            "cross_track_error_m",
            "body_center_cross_track_error_m",
            "front_axle_cross_track_error_m",
            "rear_axle_cross_track_error_m",
            "front_left_wheel_cross_track_error_m",
            "front_right_wheel_cross_track_error_m",
            "rear_left_wheel_cross_track_error_m",
            "rear_right_wheel_cross_track_error_m",
            "curve_inner_side",
            "inner_wheel_cross_track_error_m",
            "heading_error_rad",
            "distance_to_next_waypoint_m",
            "local_curvature_inv_m",
            "preview_curvature_inv_m",
            "upcoming_curvature_inv_m",
            "local_required_steering_rad",
            "preview_required_steering_rad",
            "upcoming_required_steering_rad",
            "upcoming_steering_exceeds_limit",
            "controller_mode",
            "footprint_correction_target_m",
            "footprint_correction_applied_m",
            "feedforward_steering_rad",
            "feedback_steering_rad",
            "active_heading_gain",
            "curve_speed_target_mps",
            "speed_mps",
            "target_speed_mps",
            "speed_gain",
            "throttle_command",
            "steering_command_rad",
            "steering_saturated",
            "intersection_phase",
            "lane_safety_state",
            "lane_safety_gain",
            "lane_safety_confidence",
            "lane_safety_offset_normalized",
            "lane_safety_age_s",
        ])
        self.csv_file.flush()

    @staticmethod
    def _number(value):
        return f"{float(value):.9f}"

    def record(
        self,
        elapsed,
        pose,
        tracking_point,
        controller,
        speed,
        target_speed,
        speed_gain,
        throttle,
        steering,
        intersection_phase,
        lane_safety_state="DISABLED",
        lane_safety_gain=1.0,
        lane_safety_confidence=float("nan"),
        lane_safety_offset=float("nan"),
        lane_safety_age=float("nan"),
    ):
        """Record one sample when the configured logging period has elapsed."""
        elapsed = float(elapsed)
        if elapsed - self.last_sample_time + 1e-12 < self.sample_period:
            return False

        pose = np.asarray(pose, dtype=float).reshape(3)
        tracking_point = np.asarray(tracking_point, dtype=float).reshape(2)
        index = int(controller.wpi) % (controller.N - 1)
        segment_start = np.asarray(controller.wp[:, index], dtype=float)
        segment_end = np.asarray(controller.wp[:, index + 1], dtype=float)
        steering = float(steering)
        saturated = abs(steering) >= controller.maxSteeringAngle - 1e-6
        curvature = self.curvature_analyzer.metrics(index)
        footprint, footprint_errors, inner_side, inner_wheel_error = (
            self.footprint_estimator.estimate(
                pose,
                segment_start,
                segment_end,
                curvature["local_curvature"],
            )
        )

        self.writer.writerow([
            self._number(elapsed),
            self._number(time.time()),
            self.car_id,
            self.route_id,
            *[self._number(value) for value in pose],
            *[self._number(value) for value in tracking_point],
            "ekf_pose_assumed_body_center",
            *[self._number(value) for value in footprint["front_axle_center"]],
            *[self._number(value) for value in footprint["rear_axle_center"]],
            *[self._number(value) for value in footprint["front_left_wheel"]],
            *[self._number(value) for value in footprint["front_right_wheel"]],
            *[self._number(value) for value in footprint["rear_left_wheel"]],
            *[self._number(value) for value in footprint["rear_right_wheel"]],
            index,
            *[self._number(value) for value in segment_start],
            *[self._number(value) for value in segment_end],
            *[self._number(value) for value in controller.p_ref],
            self._number(controller.th_ref),
            self._number(controller.cross_track_error),
            self._number(footprint_errors["body_center"]),
            self._number(footprint_errors["front_axle_center"]),
            self._number(footprint_errors["rear_axle_center"]),
            self._number(footprint_errors["front_left_wheel"]),
            self._number(footprint_errors["front_right_wheel"]),
            self._number(footprint_errors["rear_left_wheel"]),
            self._number(footprint_errors["rear_right_wheel"]),
            inner_side,
            self._number(inner_wheel_error),
            self._number(controller.heading_error),
            self._number(controller.distance_to_next_waypoint),
            self._number(curvature["local_curvature"]),
            self._number(curvature["preview_curvature"]),
            self._number(curvature["upcoming_curvature"]),
            self._number(curvature["local_required_steering"]),
            self._number(curvature["preview_required_steering"]),
            self._number(curvature["upcoming_required_steering"]),
            int(
                abs(curvature["upcoming_required_steering"])
                > self.maximum_steering_angle
            ),
            getattr(controller, "control_mode", "baseline"),
            self._number(getattr(
                controller, "footprint_correction_target", 0.0
            )),
            self._number(getattr(
                controller, "footprint_correction_applied", 0.0
            )),
            self._number(getattr(controller, "feedforward_steering", 0.0)),
            self._number(getattr(controller, "feedback_steering", steering)),
            self._number(getattr(controller, "active_heading_gain", 1.0)),
            self._number(getattr(controller, "curve_speed_target", target_speed)),
            self._number(speed),
            self._number(target_speed),
            self._number(speed_gain),
            self._number(throttle),
            self._number(steering),
            int(saturated),
            intersection_phase,
            lane_safety_state,
            self._number(lane_safety_gain),
            self._number(lane_safety_confidence),
            self._number(lane_safety_offset),
            self._number(lane_safety_age),
        ])
        self.csv_file.flush()
        self.last_sample_time = elapsed
        return True

    def close(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
