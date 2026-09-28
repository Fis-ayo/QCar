"""Generate a review image for candidate Phase 0 conflict-zone geometry.

This utility does not modify road_network.json.  Its output is deliberately
labelled as a candidate so physical-map measurements can be reviewed before
Phase 1 is enabled.
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent.parent.parent
QCAR_DIR = ROOT / "qcar"
sys.path.insert(0, str(QCAR_DIR))

from road_rules import RoadNetwork  # noqa: E402
from hal.products.mats import SDCSRoadMap  # noqa: E402


def cumulative_distance(points):
    segment_lengths = np.linalg.norm(np.diff(points, axis=1), axis=0)
    return np.concatenate(([0.0], np.cumsum(segment_lengths)))


def point_upstream(points, target_index, distance):
    distances = cumulative_distance(points)
    target_distance = distances[target_index]
    desired_distance = max(0.0, target_distance - distance)
    index = int(np.searchsorted(distances, desired_distance))
    index = min(max(index, 1), points.shape[1] - 2)
    return index


def point_downstream_from_position(points, position, distance):
    node_index = int(
        np.argmin(np.linalg.norm(points.T - np.asarray(position), axis=1))
    )
    distances = cumulative_distance(points)
    desired_distance = distances[node_index] + distance
    index = int(np.searchsorted(distances, desired_distance))
    return min(max(index, 1), points.shape[1] - 2)


def stop_line(points, index, width=0.32):
    tangent = points[:, index + 1] - points[:, index - 1]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    center = points[:, index]
    return np.column_stack(
        (center - normal * width / 2, center + normal * width / 2)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=str(ROOT / "docs" / "images" / "phase0_cz1_candidate.png"),
    )
    parser.add_argument(
        "--half-extent",
        type=float,
        default=0.25,
        help="Draft half-width of the square conflict polygon in meters.",
    )
    parser.add_argument(
        "--stop-offset",
        type=float,
        default=0.08,
        help="Draft stop-line distance downstream of each approach node.",
    )
    args = parser.parse_args()

    network = RoadNetwork(QCAR_DIR / "road_network.json")
    roadmap = SDCSRoadMap(leftHandTraffic=False)

    route_11 = roadmap.generate_path(
        list(network.get_route("START_NODE_11"))
    )
    route_0 = roadmap.generate_path(
        list(network.get_route("START_NODE_0"))
    )
    south_approach = roadmap.find_shortest_path(1, 13)

    pairwise_distance = np.linalg.norm(
        route_11.T[:, None, :] - route_0.T[None, :, :],
        axis=2,
    )
    index_11, index_0 = np.unravel_index(
        np.argmin(pairwise_distance), pairwise_distance.shape
    )
    crossing = (route_11[:, index_11] + route_0[:, index_0]) / 2

    node_12_position = roadmap.get_node_pose(12).squeeze()[:2]
    node_6_position = roadmap.get_node_pose(6).squeeze()[:2]
    node_1_position = roadmap.get_node_pose(1).squeeze()[:2]
    stop_index_11 = point_downstream_from_position(
        route_11,
        node_12_position,
        args.stop_offset,
    )
    stop_index_0 = point_downstream_from_position(
        route_0,
        node_6_position,
        args.stop_offset,
    )
    stop_index_south = point_downstream_from_position(
        south_approach,
        node_1_position,
        args.stop_offset,
    )
    stop_11 = stop_line(route_11, stop_index_11)
    stop_0 = stop_line(route_0, stop_index_0)
    stop_south = stop_line(south_approach, stop_index_south)

    half = args.half_extent
    polygon = np.array(
        [
            [crossing[0] - half, crossing[1] - half],
            [crossing[0] + half, crossing[1] - half],
            [crossing[0] + half, crossing[1] + half],
            [crossing[0] - half, crossing[1] + half],
        ]
    )

    figure, axis = plt.subplots(figsize=(8, 9))
    axis.plot(route_11[0], route_11[1], color="tab:blue", label="Start node 11")
    axis.plot(route_0[0], route_0[1], color="tab:orange", label="Start node 0")
    axis.plot(
        south_approach[0],
        south_approach[1],
        color="tab:green",
        linestyle="--",
        label="Future south approach 1→13",
    )

    for node_id, node in enumerate(roadmap.nodes):
        x, y = node.pose[:2, 0]
        axis.scatter(x, y, color="black", s=10, zorder=4)
        axis.text(x + 0.035, y + 0.035, str(node_id), fontsize=8)

    closed_polygon = np.vstack((polygon, polygon[0]))
    axis.fill(
        closed_polygon[:, 0],
        closed_polygon[:, 1],
        color="purple",
        alpha=0.2,
        label="CZ1 candidate polygon",
    )
    axis.plot(stop_11[0], stop_11[1], color="red", linewidth=4)
    axis.plot(stop_0[0], stop_0[1], color="red", linewidth=4)
    axis.plot(stop_south[0], stop_south[1], color="red", linewidth=4)
    axis.scatter(*crossing, marker="x", color="purple", s=100, zorder=5)

    axis.annotate(
        "Candidate stop line: node 11 route",
        stop_11.mean(axis=1),
        xytext=(10, 12),
        textcoords="offset points",
        fontsize=8,
    )
    axis.annotate(
        "Candidate stop line: node 0 route",
        stop_0.mean(axis=1),
        xytext=(10, -18),
        textcoords="offset points",
        fontsize=8,
    )
    axis.annotate(
        "Candidate stop line: south approach",
        stop_south.mean(axis=1),
        xytext=(10, -18),
        textcoords="offset points",
        fontsize=8,
    )

    axis.set_title("Phase 0 review: CZ1 candidate geometry (not active)")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)

    print(f"Saved {output_path}")
    print(f"Crossing center: [{crossing[0]:.4f}, {crossing[1]:.4f}]")
    print(f"Minimum route separation: {pairwise_distance[index_11, index_0]:.4f} m")
    print(f"Candidate polygon: {polygon.round(4).tolist()}")
    print(f"Node 11 route stop line: {stop_11.T.round(4).tolist()}")
    print(f"Node 0 route stop line: {stop_0.T.round(4).tolist()}")
    print(f"South approach stop line: {stop_south.T.round(4).tolist()}")


if __name__ == "__main__":
    main()
