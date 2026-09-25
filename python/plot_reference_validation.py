"""Overlay repeated QCar tracking/footprint data on the SDCS road graph.

This is a diagnostic plot, not painted-lane ground truth.  It compares the
reference presented to the controller with estimated body and wheel paths in
the common SDCS coordinate system.
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from hal.products.mats import SDCSRoadMap  # noqa: E402
from path_manager import PathManager  # noqa: E402
from road_rules import RoadNetwork  # noqa: E402


def prepared_route():
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    network = RoadNetwork(ROOT / "qcar" / "road_network.json")
    nodes = network.get_route("START_NODE_0")
    raw = roadmap.generate_path(list(nodes))
    result = PathManager(raw, nodes, roadmap).prepare()
    if not result.valid:
        raise RuntimeError("START_NODE_0 reference route is invalid.")
    return roadmap, nodes, result.waypoints


def plot_road_graph(axis, roadmap):
    for edge in roadmap.edges:
        points = np.asarray(edge.waypoints)
        axis.plot(
            points[0], points[1], color="0.82", linewidth=0.7,
            zorder=0,
        )
    for index, node in enumerate(roadmap.nodes):
        pose = np.asarray(node.pose).squeeze()
        if index in (2, 3, 4, 5, 6, 14, 15):
            axis.scatter(pose[0], pose[1], color="0.25", s=18, zorder=7)
            axis.annotate(
                str(index), pose[:2], xytext=(4, 4),
                textcoords="offset points", fontsize=8,
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "trial_root",
        help="Directory containing trial_01 ... trial_05 folders.",
    )
    parser.add_argument(
        "--output",
        default="reference_validation_151_node2_to_node4.png",
    )
    parser.add_argument("--minimum-waypoint", type=int, default=200)
    parser.add_argument("--maximum-waypoint", type=int, default=420)
    args = parser.parse_args()

    trial_root = Path(args.trial_root)
    files = sorted(trial_root.glob("trial_*/tracking_diagnostics.csv"))
    if not files:
        raise FileNotFoundError(
            f"No trial_*/tracking_diagnostics.csv under {trial_root}"
        )

    roadmap, route_nodes, waypoints = prepared_route()
    lane_center_separation = (438.0 - 305.5) * 0.002035
    route_to_divider = lane_center_separation / 2.0
    figure, (overview, detail) = plt.subplots(1, 2, figsize=(15, 7))

    for axis in (overview, detail):
        plot_road_graph(axis, roadmap)
        axis.plot(
            waypoints[0], waypoints[1], color="tab:blue", linewidth=1.2,
            label="Prepared controller route", zorder=2,
        )
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.2)
        axis.set_xlabel("SDCS X (m)")
        axis.set_ylabel("SDCS Y (m)")

    colors = plt.cm.viridis(np.linspace(0.08, 0.92, len(files)))
    selected_frames = []
    for color, csv_file in zip(colors, files):
        data = pd.read_csv(csv_file)
        selected = data[
            (data["waypoint_index"] >= args.minimum_waypoint)
            & (data["waypoint_index"] <= args.maximum_waypoint)
        ].copy()
        if selected.empty:
            continue
        selected_frames.append(selected)
        label = csv_file.parent.name
        overview.plot(
            selected["ekf_x_m"], selected["ekf_y_m"], color=color,
            linewidth=1.0, alpha=0.85, label=f"{label} body center",
            zorder=3,
        )
        detail.plot(
            selected["ekf_x_m"], selected["ekf_y_m"], color=color,
            linewidth=1.1, alpha=0.9, label=f"{label} body center",
            zorder=3,
        )

    if not selected_frames:
        raise ValueError("No samples exist in the requested waypoint range.")

    combined = pd.concat(selected_frames, ignore_index=True)
    # Show the footprint envelope without visually duplicating all five nearly
    # coincident wheel tracks.  Median values in small waypoint bins summarize
    # the repeatable baseline geometry.
    combined["waypoint_bin"] = (combined["waypoint_index"] // 5) * 5
    median = combined.groupby("waypoint_bin", as_index=False).median(
        numeric_only=True
    )

    # For the right-hand-traffic node-2-to-node-4 outer curve, the neighboring
    # lane lies to the left of travel. The radius constants in SDCSRoadMap give
    # an approximate centerline separation. This derived midpoint is useful as
    # a hypothesis layer, but is not a measured painted-line coordinate.
    segment = waypoints[:, args.minimum_waypoint:args.maximum_waypoint + 1]
    tangent = np.gradient(segment, axis=1)
    tangent_norm = np.linalg.norm(tangent, axis=0)
    valid_tangent = tangent_norm > 1e-9
    tangent[:, valid_tangent] /= tangent_norm[valid_tangent]
    left_normal = np.vstack((-tangent[1], tangent[0]))
    approximate_divider = segment + route_to_divider * left_normal
    neighboring_center = segment + lane_center_separation * left_normal
    detail.plot(
        approximate_divider[0], approximate_divider[1],
        color="magenta", linewidth=2.0, linestyle="-.",
        label="Approx. lane midpoint (map-derived, unmeasured)", zorder=4,
    )
    detail.plot(
        neighboring_center[0], neighboring_center[1],
        color="0.35", linewidth=1.2, linestyle="--",
        label="Approx. neighboring lane center", zorder=4,
    )
    detail.plot(
        median["tracking_x_m"], median["tracking_y_m"], color="black",
        linewidth=1.5, linestyle="--", label="Median tracking point",
        zorder=5,
    )
    detail.plot(
        median["front_left_wheel_x_m"], median["front_left_wheel_y_m"],
        color="tab:red", linewidth=1.5, label="Median front-left wheel",
        zorder=5,
    )
    detail.plot(
        median["front_right_wheel_x_m"], median["front_right_wheel_y_m"],
        color="tab:orange", linewidth=1.5, label="Median front-right wheel",
        zorder=5,
    )
    detail.plot(
        median["rear_left_wheel_x_m"], median["rear_left_wheel_y_m"],
        color="tab:red", linewidth=1.0, linestyle=":",
        label="Median rear-left wheel", zorder=5,
    )
    detail.plot(
        median["rear_right_wheel_x_m"], median["rear_right_wheel_y_m"],
        color="tab:orange", linewidth=1.0, linestyle=":",
        label="Median rear-right wheel", zorder=5,
    )

    overview.set_title(
        "Repeated .151 trajectories on SDCS road graph\n"
        "(graph lines are lane references, not painted boundaries)"
    )
    overview.legend(fontsize=7, loc="best")

    x_values = np.concatenate([
        combined["ekf_x_m"].to_numpy(),
        combined["front_left_wheel_x_m"].to_numpy(),
        combined["front_right_wheel_x_m"].to_numpy(),
    ])
    y_values = np.concatenate([
        combined["ekf_y_m"].to_numpy(),
        combined["front_left_wheel_y_m"].to_numpy(),
        combined["front_right_wheel_y_m"].to_numpy(),
    ])
    margin = 0.22
    detail.set_xlim(float(np.nanmin(x_values) - margin),
                    float(np.nanmax(x_values) + margin))
    detail.set_ylim(float(np.nanmin(y_values) - margin),
                    float(np.nanmax(y_values) + margin))
    detail.set_title(
        f"Node-2-to-node-4 diagnostic region\n"
        f"waypoints {args.minimum_waypoint}–{args.maximum_waypoint}"
    )
    detail.legend(fontsize=7, loc="best")

    figure.suptitle(
        "QCar .151 reference and estimated vehicle-footprint validation",
        fontsize=13,
    )
    figure.tight_layout()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=200)
    print(f"Route nodes: {list(route_nodes)}")
    print(f"Trials plotted: {len(selected_frames)}")
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
