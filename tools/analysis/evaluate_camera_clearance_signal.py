"""Evaluate CSI image features as non-controlling lane-clearance warnings."""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


WHEEL_COLUMNS = [
    "front_left_wheel_cross_track_error_m",
    "front_right_wheel_cross_track_error_m",
    "rear_left_wheel_cross_track_error_m",
    "rear_right_wheel_cross_track_error_m",
]

FEATURES = [
    "center_offset_normalized",
    "heading_error_rad_lane",
    "confidence",
    "yellow_offset_normalized",
    "yellow_heading_rad",
    "yellow_curvature",
    "yellow_confidence",
    "yellow_row_coverage",
    "yellow_fit_rmse_px",
    "yellow_temporal_jump_px",
]


def synchronize(tracking_file, lane_file, boundary, minimum_wp, maximum_wp):
    tracking = pd.read_csv(tracking_file).sort_values("wall_time_s")
    tracking = tracking[
        (tracking["waypoint_index"] >= minimum_wp)
        & (tracking["waypoint_index"] <= maximum_wp)
    ].copy()
    lane = pd.read_csv(lane_file).sort_values("wall_time_s")
    joined = pd.merge_asof(
        tracking,
        lane,
        on="wall_time_s",
        direction="nearest",
        tolerance=0.08,
        suffixes=("_tracking", "_lane"),
    ).dropna(subset=["frame_index"])
    wheels = joined[WHEEL_COLUMNS].to_numpy(dtype=float)
    lower = np.nanmin(wheels, axis=1)
    upper = np.nanmax(wheels, axis=1)
    joined["negative_violation"] = lower < -boundary
    joined["positive_violation"] = upper > boundary
    joined["inside_lane"] = ~(
        joined["negative_violation"] | joined["positive_violation"]
    )
    return joined


def best_threshold(values, labels):
    valid = np.isfinite(values)
    values = values[valid]
    labels = labels[valid].astype(bool)
    positives = np.count_nonzero(labels)
    negatives = np.count_nonzero(~labels)
    if positives == 0 or negatives == 0 or len(np.unique(values)) < 2:
        return None
    candidates = np.unique(np.quantile(values, np.linspace(0.02, 0.98, 97)))
    best = None
    for direction in ("greater", "less"):
        for threshold in candidates:
            predicted = (
                values >= threshold if direction == "greater"
                else values <= threshold
            )
            sensitivity = np.mean(predicted[labels])
            specificity = np.mean(~predicted[~labels])
            balanced = 0.5 * (sensitivity + specificity)
            result = (balanced, sensitivity, specificity, direction, threshold)
            if best is None or result[0] > best[0]:
                best = result
    return best


def evaluate_rule(values, labels, direction, threshold):
    valid = np.isfinite(values)
    values = values[valid]
    labels = labels[valid].astype(bool)
    if not np.any(labels) or not np.any(~labels):
        return None
    predicted = values >= threshold if direction == "greater" else values <= threshold
    sensitivity = np.mean(predicted[labels])
    specificity = np.mean(~predicted[~labels])
    return 0.5 * (sensitivity + specificity), sensitivity, specificity


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "pairs", nargs="+", help="Pairs formatted tracking.csv::lane.csv"
    )
    parser.add_argument("--boundary-distance", type=float, default=0.13335)
    parser.add_argument("--minimum-waypoint", type=int, default=200)
    parser.add_argument("--maximum-waypoint", type=int, default=400)
    args = parser.parse_args()

    runs = []
    for pair in args.pairs:
        tracking_name, lane_name = pair.split("::", 1)
        name = Path(tracking_name).parent.name
        frame = synchronize(
            tracking_name,
            lane_name,
            args.boundary_distance,
            args.minimum_waypoint,
            args.maximum_waypoint,
        )
        frame["run_name"] = name
        runs.append(frame)
        print(
            f"{name}: samples={len(frame)}, "
            f"inside={100*np.mean(frame['inside_lane']):.1f}%, "
            f"node5={100*np.mean(frame['negative_violation']):.1f}%, "
            f"outer={100*np.mean(frame['positive_violation']):.1f}%"
        )

    combined = pd.concat(runs, ignore_index=True)
    combined["seconds_to_run_end"] = combined.groupby("run_name")[
        "wall_time_s"
    ].transform("max") - combined["wall_time_s"]
    targets = (
        ("node5", "negative_violation"),
        ("outer", "positive_violation"),
        ("any_boundary", None),
    )
    print("\nAll-data candidate thresholds (diagnostic only):")
    for target_name, column in targets:
        labels = (
            (~combined["inside_lane"]).to_numpy(dtype=bool)
            if column is None else combined[column].to_numpy(dtype=bool)
        )
        ranked = []
        for feature in FEATURES:
            if feature not in combined:
                continue
            result = best_threshold(
                combined[feature].to_numpy(dtype=float), labels
            )
            if result is not None:
                ranked.append((result[0], feature, result))
        print(f"  {target_name}:")
        for _, feature, result in sorted(ranked, reverse=True)[:5]:
            balanced, sensitivity, specificity, direction, threshold = result
            print(
                f"    {feature}: {direction} {threshold:.4f}; "
                f"balanced={balanced:.3f}, sensitivity={sensitivity:.3f}, "
                f"specificity={specificity:.3f}"
            )

    print("\nLeave-one-run-out yellow-offset validation:")
    for target_name, column in targets:
        print(f"  {target_name}:")
        for held_out in sorted(combined["run_name"].unique()):
            training = combined[combined["run_name"] != held_out]
            test = combined[combined["run_name"] == held_out]
            training_labels = (
                (~training["inside_lane"]).to_numpy(dtype=bool)
                if column is None else training[column].to_numpy(dtype=bool)
            )
            result = best_threshold(
                training["yellow_offset_normalized"].to_numpy(dtype=float),
                training_labels,
            )
            if result is None:
                print(f"    hold {held_out}: insufficient training classes")
                continue
            _, _, _, direction, threshold = result
            test_labels = (
                (~test["inside_lane"]).to_numpy(dtype=bool)
                if column is None else test[column].to_numpy(dtype=bool)
            )
            score = evaluate_rule(
                test["yellow_offset_normalized"].to_numpy(dtype=float),
                test_labels,
                direction,
                threshold,
            )
            if score is None:
                print(f"    hold {held_out}: only one test class")
                continue
            balanced, sensitivity, specificity = score
            print(
                f"    hold {held_out}: {direction} {threshold:.4f}; "
                f"balanced={balanced:.3f}, sensitivity={sensitivity:.3f}, "
                f"specificity={specificity:.3f}"
            )

    # Every supplied run was stopped by the operator after observing entry
    # into the node-5 lane. Compare the final one-second window with an earlier
    # 3-6 second reference window. This is an event proxy, not a frame-accurate
    # physical crossing annotation.
    stop_proxy = combined[
        (combined["seconds_to_run_end"] <= 1.0)
        | (
            (combined["seconds_to_run_end"] >= 3.0)
            & (combined["seconds_to_run_end"] <= 6.0)
        )
    ].copy()
    stop_proxy["operator_stop_window"] = (
        stop_proxy["seconds_to_run_end"] <= 1.0
    )
    print("\nOperator-stop proxy (final 1 s versus 3-6 s earlier):")
    ranked = []
    labels = stop_proxy["operator_stop_window"].to_numpy(dtype=bool)
    for feature in FEATURES:
        if feature not in stop_proxy:
            continue
        result = best_threshold(
            stop_proxy[feature].to_numpy(dtype=float), labels
        )
        if result is not None:
            ranked.append((result[0], feature, result))
    for _, feature, result in sorted(ranked, reverse=True)[:7]:
        balanced, sensitivity, specificity, direction, threshold = result
        print(
            f"  {feature}: {direction} {threshold:.4f}; "
            f"balanced={balanced:.3f}, sensitivity={sensitivity:.3f}, "
            f"specificity={specificity:.3f}"
        )

    print("\nLeave-one-run-out operator-stop proxy, yellow offset:")
    for held_out in sorted(stop_proxy["run_name"].unique()):
        training = stop_proxy[stop_proxy["run_name"] != held_out]
        test = stop_proxy[stop_proxy["run_name"] == held_out]
        result = best_threshold(
            training["yellow_offset_normalized"].to_numpy(dtype=float),
            training["operator_stop_window"].to_numpy(dtype=bool),
        )
        if result is None:
            print(f"  hold {held_out}: insufficient training classes")
            continue
        _, _, _, direction, threshold = result
        score = evaluate_rule(
            test["yellow_offset_normalized"].to_numpy(dtype=float),
            test["operator_stop_window"].to_numpy(dtype=bool),
            direction,
            threshold,
        )
        if score is None:
            print(f"  hold {held_out}: only one test class")
            continue
        balanced, sensitivity, specificity = score
        print(
            f"  hold {held_out}: {direction} {threshold:.4f}; "
            f"balanced={balanced:.3f}, sensitivity={sensitivity:.3f}, "
            f"specificity={specificity:.3f}"
        )


if __name__ == "__main__":
    main()
