"""Replay continuous corridor-centering against a tracking diagnostics CSV."""

import argparse
import csv
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "qcar"))

from lane_guidance import ContinuousCorridorAdvisor  # noqa: E402


def number(row, field):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError):
        return float("nan")
    return value


def observation(row):
    return {
        "corridor_detected": row.get("corridor_detected") == "1",
        "corridor_confidence": number(row, "corridor_confidence"),
        "corridor_outer_source": row.get("corridor_outer_source", "none"),
        "corridor_offset_normalized": number(
            row, "corridor_offset_normalized"
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("tracking_csv", type=Path)
    parser.add_argument("--jump-threshold-m", type=float, default=0.04)
    args = parser.parse_args()

    with args.tracking_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    advisor = ContinuousCorridorAdvisor()
    previous_time = None
    previous_cross_track = None
    reliable = 0
    positive = 0
    negative = 0
    maximum_candidate = 0.0
    discontinuities = []
    for row in rows:
        elapsed = number(row, "elapsed_s")
        dt = 0.0 if previous_time is None else max(0.0, elapsed - previous_time)
        previous_time = elapsed
        result = advisor.update(
            observation(row),
            number(row, "local_curvature_inv_m"),
            number(row, "waypoint_steering_command_rad"),
            dt,
            motion_allowed=row.get("boundary_safety_state") != "STOP",
        )
        if result["reliable"]:
            reliable += 1
        candidate = result["filtered_candidate_rad"]
        positive += candidate > 1e-6
        negative += candidate < -1e-6
        maximum_candidate = max(maximum_candidate, abs(candidate))

        cross_track = number(row, "cross_track_error_m")
        moving = (
            number(row, "speed_mps") > 0.03
            and row.get("boundary_safety_state") != "STOP"
        )
        if (
            moving
            and previous_cross_track is not None
            and math.isfinite(cross_track)
            and abs(cross_track - previous_cross_track)
            >= args.jump_threshold_m
        ):
            discontinuities.append({
                "elapsed_s": elapsed,
                "waypoint_index": int(number(row, "waypoint_index")),
                "cross_track_before_m": previous_cross_track,
                "cross_track_after_m": cross_track,
                "jump_m": cross_track - previous_cross_track,
                "waypoint_steering_rad": number(
                    row, "waypoint_steering_command_rad"
                ),
                "local_curvature_inv_m": number(
                    row, "local_curvature_inv_m"
                ),
            })
        if math.isfinite(cross_track):
            previous_cross_track = cross_track

    print(f"file={args.tracking_csv}")
    print(f"rows={len(rows)} reliable_corridor_rows={reliable}")
    print(
        f"continuous_candidate_positive_rows={positive} "
        f"negative_rows={negative} max_abs_rad={maximum_candidate:.6f}"
    )
    print(f"cross_track_jumps={len(discontinuities)}")
    for event in discontinuities:
        print(
            "jump "
            f"t={event['elapsed_s']:.3f}s "
            f"wp={event['waypoint_index']} "
            f"{event['cross_track_before_m']:.4f}->"
            f"{event['cross_track_after_m']:.4f}m "
            f"delta={event['jump_m']:.4f}m "
            f"steering={event['waypoint_steering_rad']:.4f}rad "
            f"curvature={event['local_curvature_inv_m']:.4f}/m"
        )


if __name__ == "__main__":
    main()
