"""Plot raw and prepared waypoint routes for offline inspection."""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from hal.products.mats import SDCSRoadMap  # noqa: E402
from path_manager import PathManager  # noqa: E402
from road_rules import RoadNetwork  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=str(ROOT / "validated_routes.png"),
    )
    args = parser.parse_args()

    roadmap = SDCSRoadMap(leftHandTraffic=False)
    network = RoadNetwork(ROOT / "qcar" / "road_network.json")
    route_ids = ("START_NODE_11", "START_NODE_0")
    figure, axes = plt.subplots(1, 2, figsize=(13, 6))

    for axis, route_id in zip(axes, route_ids):
        nodes = network.get_route(route_id)
        raw = roadmap.generate_path(list(nodes))
        result = PathManager(raw, nodes, roadmap).prepare()
        axis.plot(
            raw[0],
            raw[1],
            color="tab:red",
            linestyle="--",
            linewidth=1.5,
            label="Raw path",
        )
        axis.plot(
            result.waypoints[0],
            result.waypoints[1],
            color="tab:blue",
            linewidth=1.0,
            label="Prepared path",
        )
        for node in dict.fromkeys(nodes):
            pose = roadmap.get_node_pose(node).squeeze()
            axis.scatter(pose[0], pose[1], color="black", s=22, zorder=4)
            axis.annotate(str(node), pose[:2], xytext=(4, 4), textcoords="offset points")
        for repair in result.repairs:
            point = np.asarray(repair["removed_point"])
            axis.scatter(
                point[0],
                point[1],
                marker="x",
                s=90,
                linewidth=2,
                color="magenta",
                label="Removed spike",
                zorder=5,
            )
        axis.set_title(
            f"{route_id}\nvalid={result.valid}, repairs={len(result.repairs)}"
        )
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.axis("equal")
        axis.grid(True, alpha=0.25)
        axis.legend()

    figure.suptitle("Raw versus validated SDCS routes")
    figure.tight_layout()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
