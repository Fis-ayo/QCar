"""Measurement-only diagnostics for QCar LiDAR/GPS localization.

This module does not filter measurements or change vehicle control. It records
the scan-matched pose received from QCarGPS beside the EKF state and preserves
full recent LiDAR scans when an implausibly large pose innovation is observed.
"""

import csv
import os
import time
from collections import deque

import numpy as np


class LocalizationDiagnostics:
    def __init__(
        self,
        car_id,
        csv_path="localization_diagnostics.csv",
        event_directory="localization_events",
        position_event_threshold=0.35,
        heading_event_threshold=np.deg2rad(45.0),
        event_cooldown=2.0,
        lidar_history_size=20,
    ):
        self.car_id = car_id
        self.csv_path = csv_path
        self.event_directory = event_directory
        self.position_event_threshold = float(position_event_threshold)
        self.heading_event_threshold = float(heading_event_threshold)
        self.event_cooldown = float(event_cooldown)
        self.lidar_history = deque(maxlen=int(lidar_history_size))
        self.last_event_time = float("-inf")
        self.event_count = 0
        self.last_raw_pose = np.full(3, np.nan)
        self.last_position_innovation = float("nan")
        self.last_heading_innovation = float("nan")
        self.last_gps_time = float("-inf")
        self.last_lidar_valid = 0
        self.last_lidar_median = float("nan")

        os.makedirs(self.event_directory, exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "elapsed_s",
            "wall_time_s",
            "car_id",
            "raw_x_m",
            "raw_y_m",
            "raw_heading_rad",
            "ekf_pre_x_m",
            "ekf_pre_y_m",
            "ekf_pre_heading_rad",
            "ekf_post_x_m",
            "ekf_post_y_m",
            "ekf_post_heading_rad",
            "position_innovation_m",
            "heading_innovation_rad",
            "motor_speed_mps",
            "steering_command_rad",
            "gyro_z_rad_s",
            "lidar_valid_points",
            "lidar_min_m",
            "lidar_median_m",
            "lidar_max_m",
            "event_capture",
        ])
        self.csv_file.flush()

    def update_lidar(self, elapsed, angles, distances):
        angles = np.asarray(angles, dtype=float).copy()
        distances = np.asarray(distances, dtype=float).copy()
        valid = np.isfinite(distances) & (distances > 0.0)
        valid_distances = distances[valid]
        self.last_lidar_valid = int(valid_distances.size)
        if valid_distances.size:
            lidar_min = float(np.min(valid_distances))
            lidar_median = float(np.median(valid_distances))
            lidar_max = float(np.max(valid_distances))
        else:
            lidar_min = lidar_median = lidar_max = float("nan")
        self.last_lidar_median = lidar_median
        self.lidar_history.append({
            "elapsed": float(elapsed),
            "angles": angles,
            "distances": distances,
            "valid_points": self.last_lidar_valid,
            "minimum": lidar_min,
            "median": lidar_median,
            "maximum": lidar_max,
        })

    def record_gps_update(
        self,
        elapsed,
        raw_pose,
        ekf_pre,
        ekf_post,
        position_innovation,
        heading_innovation,
        motor_speed,
        steering_command,
        gyro_z,
    ):
        raw_pose = np.asarray(raw_pose, dtype=float).reshape(3)
        ekf_pre = np.asarray(ekf_pre, dtype=float).reshape(3)
        ekf_post = np.asarray(ekf_post, dtype=float).reshape(3)
        self.last_raw_pose = raw_pose.copy()
        self.last_position_innovation = float(position_innovation)
        self.last_heading_innovation = float(heading_innovation)
        self.last_gps_time = float(elapsed)

        latest_lidar = self.lidar_history[-1] if self.lidar_history else None
        event_capture = (
            self.last_position_innovation >= self.position_event_threshold
            or abs(self.last_heading_innovation) >= self.heading_event_threshold
        ) and elapsed - self.last_event_time >= self.event_cooldown

        if event_capture:
            self._save_event(
                elapsed,
                raw_pose,
                ekf_pre,
                ekf_post,
                motor_speed,
                steering_command,
                gyro_z,
            )
            self.last_event_time = float(elapsed)

        self.writer.writerow([
            f"{elapsed:.6f}",
            f"{time.time():.6f}",
            self.car_id,
            *[f"{value:.9f}" for value in raw_pose],
            *[f"{value:.9f}" for value in ekf_pre],
            *[f"{value:.9f}" for value in ekf_post],
            f"{self.last_position_innovation:.9f}",
            f"{self.last_heading_innovation:.9f}",
            f"{float(motor_speed):.9f}",
            f"{float(steering_command):.9f}",
            f"{float(gyro_z):.9f}",
            0 if latest_lidar is None else latest_lidar["valid_points"],
            "nan" if latest_lidar is None else f"{latest_lidar['minimum']:.9f}",
            "nan" if latest_lidar is None else f"{latest_lidar['median']:.9f}",
            "nan" if latest_lidar is None else f"{latest_lidar['maximum']:.9f}",
            int(event_capture),
        ])
        self.csv_file.flush()

    def _save_event(
        self,
        elapsed,
        raw_pose,
        ekf_pre,
        ekf_post,
        motor_speed,
        steering_command,
        gyro_z,
    ):
        self.event_count += 1
        filename = os.path.join(
            self.event_directory,
            f"event_{self.event_count:03d}_{elapsed:.3f}s.npz",
        )
        if self.lidar_history:
            scan_times = np.asarray(
                [scan["elapsed"] for scan in self.lidar_history], dtype=float
            )
            angles = np.stack(
                [scan["angles"] for scan in self.lidar_history], axis=0
            )
            distances = np.stack(
                [scan["distances"] for scan in self.lidar_history], axis=0
            )
        else:
            scan_times = np.empty(0, dtype=float)
            angles = np.empty((0, 0), dtype=float)
            distances = np.empty((0, 0), dtype=float)
        np.savez_compressed(
            filename,
            elapsed=np.asarray([elapsed], dtype=float),
            raw_pose=np.asarray(raw_pose, dtype=float),
            ekf_pre=np.asarray(ekf_pre, dtype=float),
            ekf_post=np.asarray(ekf_post, dtype=float),
            position_innovation=np.asarray(
                [self.last_position_innovation], dtype=float
            ),
            heading_innovation=np.asarray(
                [self.last_heading_innovation], dtype=float
            ),
            motor_speed=np.asarray([motor_speed], dtype=float),
            steering_command=np.asarray([steering_command], dtype=float),
            gyro_z=np.asarray([gyro_z], dtype=float),
            lidar_scan_times=scan_times,
            lidar_angles=angles,
            lidar_distances=distances,
        )
        print(
            "[LOCALIZATION] Captured outlier event "
            f"{filename}: position innovation="
            f"{self.last_position_innovation:.3f} m, heading innovation="
            f"{np.degrees(self.last_heading_innovation):.1f} deg",
            flush=True,
        )

    def gps_age(self, elapsed):
        if not np.isfinite(self.last_gps_time):
            return float("inf")
        return max(0.0, float(elapsed) - self.last_gps_time)

    def close(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
