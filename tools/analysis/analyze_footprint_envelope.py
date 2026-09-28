"""Evaluate whether the estimated QCar wheel envelope fits inside a lane."""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


WHEEL_ERROR_COLUMNS = [
    "front_left_wheel_cross_track_error_m",
    "front_right_wheel_cross_track_error_m",
    "rear_left_wheel_cross_track_error_m",
    "rear_right_wheel_cross_track_error_m",
]


def summarize(csv_file, boundary_distance, minimum_waypoint, maximum_waypoint):
    data = pd.read_csv(csv_file)
    selected = data[
        (data["waypoint_index"] >= minimum_waypoint)
        & (data["waypoint_index"] <= maximum_waypoint)
    ]
    if selected.empty:
        raise ValueError(f"No selected waypoint samples in {csv_file}")
    wheel_errors = selected[WHEEL_ERROR_COLUMNS].to_numpy(dtype=float)
    lower = np.nanmin(wheel_errors, axis=1)
    upper = np.nanmax(wheel_errors, axis=1)
    width = upper - lower
    centered_shift = -(upper + lower) / 2.0
    fit = width <= 2.0 * boundary_distance
    return {
        "samples": len(selected),
        "fit_percent": 100.0 * np.mean(fit),
        "negative_boundary_violation_percent": 100.0 * np.mean(
            lower < -boundary_distance
        ),
        "positive_boundary_violation_percent": 100.0 * np.mean(
            upper > boundary_distance
        ),
        "mean_width_m": float(np.mean(width)),
        "maximum_width_m": float(np.max(width)),
        "mean_centered_shift_m": float(np.mean(centered_shift)),
        "p95_absolute_centered_shift_m": float(
            np.percentile(np.abs(centered_shift), 95)
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_files", nargs="+")
    parser.add_argument("--boundary-distance", type=float, default=0.13335)
    parser.add_argument("--minimum-waypoint", type=int, default=200)
    parser.add_argument("--maximum-waypoint", type=int, default=400)
    args = parser.parse_args()

    print(
        "Boundary distance from lane center: "
        f"{args.boundary_distance:.5f} m"
    )
    for filename in args.csv_files:
        metrics = summarize(
            filename,
            args.boundary_distance,
            args.minimum_waypoint,
            args.maximum_waypoint,
        )
        print(f"\n{Path(filename).parent.name}: {filename}")
        print(f"  samples: {metrics['samples']}")
        print(f"  complete wheel envelope fits: {metrics['fit_percent']:.1f}%")
        print(
            "  negative-side violation: "
            f"{metrics['negative_boundary_violation_percent']:.1f}%"
        )
        print(
            "  positive-side violation: "
            f"{metrics['positive_boundary_violation_percent']:.1f}%"
        )
        print(f"  mean envelope width: {metrics['mean_width_m']:.4f} m")
        print(f"  maximum envelope width: {metrics['maximum_width_m']:.4f} m")
        print(
            "  mean translation to center envelope: "
            f"{metrics['mean_centered_shift_m']:.4f} m"
        )
        print(
            "  p95 |translation to center envelope|: "
            f"{metrics['p95_absolute_centered_shift_m']:.4f} m"
        )


if __name__ == "__main__":
    main()

