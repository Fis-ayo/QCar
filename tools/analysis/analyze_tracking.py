"""Summarize and plot a QCar tracking_diagnostics.csv file."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("--output", default="tracking_analysis.png")
    args = parser.parse_args()

    data = pd.read_csv(args.csv_file)
    if data.empty:
        raise ValueError("Tracking CSV contains no samples.")

    cross_track = np.abs(data["cross_track_error_m"].to_numpy())
    heading_deg = np.abs(np.degrees(data["heading_error_rad"].to_numpy()))
    saturated = data["steering_saturated"].astype(bool).to_numpy()
    worst = int(np.argmax(cross_track))

    print(f"Samples: {len(data)}")
    print(f"Duration: {data['elapsed_s'].iloc[-1] - data['elapsed_s'].iloc[0]:.2f} s")
    print(f"Mean |cross-track|: {np.mean(cross_track):.3f} m")
    print(f"RMS cross-track: {np.sqrt(np.mean(np.square(cross_track))):.3f} m")
    print(f"Median |cross-track|: {np.median(cross_track):.3f} m")
    print(f"95th percentile |cross-track|: {np.percentile(cross_track, 95):.3f} m")
    print(f"Maximum |cross-track|: {np.max(cross_track):.3f} m")
    print(f"Mean |heading error|: {np.mean(heading_deg):.1f} deg")
    print(f"95th percentile |heading error|: {np.percentile(heading_deg, 95):.1f} deg")
    print(f"Steering saturation: {100.0 * np.mean(saturated):.1f}% of samples")
    if "upcoming_curvature_inv_m" in data:
        upcoming_curvature = np.abs(
            data["upcoming_curvature_inv_m"].to_numpy()
        )
        required_steering = np.abs(np.degrees(
            data["upcoming_required_steering_rad"].to_numpy()
        ))
        exceeds = data["upcoming_steering_exceeds_limit"].astype(bool)
        print(f"Maximum upcoming curvature: {np.max(upcoming_curvature):.3f} 1/m")
        print(f"Maximum predicted steering demand: {np.max(required_steering):.1f} deg")
        print(f"Predicted steering-limit exceedance: {100.0 * np.mean(exceeds):.1f}% of samples")
    if "controller_mode" in data:
        print(f"Controller mode: {data['controller_mode'].iloc[0]}")
    if "footprint_correction_applied_m" in data:
        correction = data["footprint_correction_applied_m"].to_numpy()
        target = data["footprint_correction_target_m"].to_numpy()
        print(
            "Maximum |footprint correction target|: "
            f"{np.max(np.abs(target)):.3f} m"
        )
        print(
            "Maximum |footprint correction applied|: "
            f"{np.max(np.abs(correction)):.3f} m"
        )
    print(
        "Worst cross-track sample: "
        f"t={data['elapsed_s'].iloc[worst]:.2f}s, "
        f"wp={int(data['waypoint_index'].iloc[worst])}, "
        f"position=({data['ekf_x_m'].iloc[worst]:.3f}, "
        f"{data['ekf_y_m'].iloc[worst]:.3f})"
    )

    figure, axes = plt.subplots(2, 1, figsize=(10, 10))
    route_axis, time_axis = axes
    route_axis.plot(data["reference_x_m"], data["reference_y_m"],
                    color="tab:blue", linewidth=1.2,
                    label="Controller reference")
    route_axis.plot(data["tracking_x_m"], data["tracking_y_m"],
                    color="tab:orange", linewidth=1.0,
                    label="Tracking point")
    route_axis.plot(data["ekf_x_m"], data["ekf_y_m"],
                    color="tab:green", linewidth=0.8,
                    label="Estimated vehicle center")
    if np.any(saturated):
        route_axis.scatter(data.loc[saturated, "tracking_x_m"],
                           data.loc[saturated, "tracking_y_m"],
                           color="red", s=8, label="Steering saturated",
                           zorder=4)
    route_axis.set_title(f"Tracking trajectory: {data['car_id'].iloc[0]}")
    route_axis.set_xlabel("X (m)")
    route_axis.set_ylabel("Y (m)")
    route_axis.axis("equal")
    route_axis.grid(True, alpha=0.25)
    route_axis.legend()

    time_axis.plot(data["elapsed_s"], data["cross_track_error_m"],
                   label="Cross-track error (m)")
    time_axis.plot(data["elapsed_s"],
                   np.degrees(data["heading_error_rad"]),
                   label="Heading error (deg)", alpha=0.8)
    time_axis.plot(data["elapsed_s"], data["steering_command_rad"],
                   label="Steering command (rad)", alpha=0.8)
    if "upcoming_required_steering_rad" in data:
        time_axis.plot(
            data["elapsed_s"], data["upcoming_required_steering_rad"],
            label="Upcoming required steering (rad)", alpha=0.7,
        )
    time_axis.set_xlabel("Elapsed time (s)")
    time_axis.grid(True, alpha=0.25)
    time_axis.legend()

    figure.tight_layout()
    output = Path(args.output)
    figure.savefig(output, dpi=180)
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
