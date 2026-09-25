"""Compare measurement-only CSI features with map-relative wheel clearance."""

import argparse

import numpy as np
import pandas as pd


WHEEL_COLUMNS = [
    "front_left_wheel_cross_track_error_m",
    "front_right_wheel_cross_track_error_m",
    "rear_left_wheel_cross_track_error_m",
    "rear_right_wheel_cross_track_error_m",
]


def correlation(first, second):
    valid = np.isfinite(first) & np.isfinite(second)
    if np.count_nonzero(valid) < 3:
        return float("nan")
    return float(np.corrcoef(first[valid], second[valid])[0, 1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("tracking_csv")
    parser.add_argument("lane_csv")
    parser.add_argument("--boundary-distance", type=float, default=0.13335)
    parser.add_argument("--minimum-waypoint", type=int, default=200)
    parser.add_argument("--maximum-waypoint", type=int, default=400)
    parser.add_argument("--maximum-time-gap", type=float, default=0.08)
    args = parser.parse_args()

    tracking = pd.read_csv(args.tracking_csv).sort_values("wall_time_s")
    tracking = tracking[
        (tracking["waypoint_index"] >= args.minimum_waypoint)
        & (tracking["waypoint_index"] <= args.maximum_waypoint)
    ].copy()
    lane = pd.read_csv(args.lane_csv).sort_values("wall_time_s")
    joined = pd.merge_asof(
        tracking,
        lane,
        on="wall_time_s",
        direction="nearest",
        tolerance=args.maximum_time_gap,
        suffixes=("_tracking", "_lane"),
    )
    joined = joined.dropna(subset=["frame_index"])
    if joined.empty:
        raise ValueError("No synchronized camera/tracking samples found.")

    wheel_errors = joined[WHEEL_COLUMNS].to_numpy(dtype=float)
    negative_edge = np.nanmin(wheel_errors, axis=1)
    positive_edge = np.nanmax(wheel_errors, axis=1)
    negative_clearance = negative_edge + args.boundary_distance
    positive_clearance = args.boundary_distance - positive_edge
    center_offset = joined["center_offset_normalized"].to_numpy(dtype=float)
    yellow_offset = joined["yellow_offset_normalized"].to_numpy(dtype=float)
    yellow_detected = joined["yellow_detected"].to_numpy(dtype=float) > 0.5

    print(f"Synchronized samples: {len(joined)}")
    print(f"Yellow detection: {100.0 * np.mean(yellow_detected):.1f}%")
    print(
        "Negative-side violation samples: "
        f"{100.0 * np.mean(negative_clearance < 0.0):.1f}%"
    )
    print(
        "Positive-side violation samples: "
        f"{100.0 * np.mean(positive_clearance < 0.0):.1f}%"
    )
    print(
        "corr(two-boundary center offset, negative clearance): "
        f"{correlation(center_offset, negative_clearance):.3f}"
    )
    print(
        "corr(two-boundary center offset, positive clearance): "
        f"{correlation(center_offset, positive_clearance):.3f}"
    )
    print(
        "corr(yellow offset, negative clearance): "
        f"{correlation(yellow_offset, negative_clearance):.3f}"
    )
    print(
        "corr(yellow offset, positive clearance): "
        f"{correlation(yellow_offset, positive_clearance):.3f}"
    )

    for label, mask in (
        ("inside lane", (negative_clearance >= 0) & (positive_clearance >= 0)),
        ("negative violation", negative_clearance < 0),
        ("positive violation", positive_clearance < 0),
    ):
        if not np.any(mask):
            continue
        print(f"{label}:")
        print(
            "  median two-boundary center offset: "
            f"{np.nanmedian(center_offset[mask]):.3f}"
        )
        print(
            "  median yellow offset: "
            f"{np.nanmedian(yellow_offset[mask]):.3f}"
        )


if __name__ == "__main__":
    main()

