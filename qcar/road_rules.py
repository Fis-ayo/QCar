"""Load and validate the physical SDCS road-network configuration.

This module contains no vehicle-control behavior.  It is the Phase 0 boundary
between map facts (JSON) and the Phase 1 intersection coordinator/controller.
"""

import json
from pathlib import Path


class RoadNetworkError(ValueError):
    """Raised when the road-network configuration is invalid."""


class RoadNetwork:
    """Validated, read-only view of the shared road-network configuration."""

    SUPPORTED_SCHEMA_VERSION = 1

    def __init__(self, path=None):
        if path is None:
            path = Path(__file__).with_name("road_network.json")
        self.path = Path(path)
        with self.path.open("r", encoding="utf-8") as config_file:
            self.data = json.load(config_file)
        self._validate()

    def _require(self, mapping, key, context):
        if key not in mapping:
            raise RoadNetworkError(f"Missing {context}.{key}")
        return mapping[key]

    @staticmethod
    def _validate_node_list(nodes, context):
        if not isinstance(nodes, list) or not all(
            isinstance(node, int) and node >= 0 for node in nodes
        ):
            raise RoadNetworkError(
                f"{context} must be a list of non-negative node numbers"
            )

    @staticmethod
    def _validate_edge(edge, context):
        if (
            not isinstance(edge, list)
            or len(edge) != 2
            or not all(isinstance(node, int) and node >= 0 for node in edge)
        ):
            raise RoadNetworkError(
                f"{context} must be a two-node list such as [12, 7]"
            )

    @staticmethod
    def _validate_polygon(polygon, context, required):
        if not isinstance(polygon, list):
            raise RoadNetworkError(f"{context} must be a list")
        if required and len(polygon) < 3:
            raise RoadNetworkError(
                f"{context} needs at least three points when phase1_ready is true"
            )
        for index, point in enumerate(polygon):
            if (
                not isinstance(point, list)
                or len(point) != 2
                or not all(isinstance(value, (int, float)) for value in point)
            ):
                raise RoadNetworkError(
                    f"{context}[{index}] must be an [x, y] coordinate"
                )

    @staticmethod
    def _validate_line(line, context, required):
        if not isinstance(line, list):
            raise RoadNetworkError(f"{context} must be a list")
        if required and len(line) != 2:
            raise RoadNetworkError(
                f"{context} requires exactly two points when phase1_ready is true"
            )
        for index, point in enumerate(line):
            if (
                not isinstance(point, list)
                or len(point) != 2
                or not all(isinstance(value, (int, float)) for value in point)
            ):
                raise RoadNetworkError(
                    f"{context}[{index}] must be an [x, y] coordinate"
                )

    def _validate(self):
        schema_version = self._require(self.data, "schema_version", "root")
        if schema_version != self.SUPPORTED_SCHEMA_VERSION:
            raise RoadNetworkError(
                f"Unsupported schema version {schema_version}; expected "
                f"{self.SUPPORTED_SCHEMA_VERSION}"
            )

        availability = self._require(
            self.data, "road_availability", "root"
        )
        disabled_nodes = self._require(
            availability, "disabled_nodes", "road_availability"
        )
        inactive_nodes = self._require(
            availability, "inactive_nodes", "road_availability"
        )
        disabled_edges = self._require(
            availability, "disabled_edges", "road_availability"
        )
        self._validate_node_list(disabled_nodes, "disabled_nodes")
        self._validate_node_list(inactive_nodes, "inactive_nodes")
        if set(disabled_nodes) & set(inactive_nodes):
            raise RoadNetworkError(
                "A node cannot be both disabled and inactive"
            )
        for index, edge in enumerate(disabled_edges):
            self._validate_edge(edge, f"disabled_edges[{index}]")

        prohibited_nodes = set(disabled_nodes) | set(inactive_nodes)
        routes = self._require(self.data, "routes", "root")
        if not isinstance(routes, dict) or not routes:
            raise RoadNetworkError("root.routes must contain at least one route")
        for route_id, route in routes.items():
            sequence = self._require(
                route, "node_sequence", f"routes.{route_id}"
            )
            self._validate_node_list(sequence, f"routes.{route_id}.node_sequence")
            if len(sequence) < 2 or sequence[0] != sequence[-1]:
                raise RoadNetworkError(
                    f"Route {route_id} must be a closed node sequence"
                )
            used_prohibited = sorted(set(sequence) & prohibited_nodes)
            if used_prohibited:
                raise RoadNetworkError(
                    f"Route {route_id} uses unavailable nodes {used_prohibited}"
                )

        zones = self._require(self.data, "conflict_zones", "root")
        if not isinstance(zones, dict):
            raise RoadNetworkError("root.conflict_zones must be an object")
        for zone_id, zone in zones.items():
            phase1_ready = bool(zone.get("phase1_ready", False))
            self._require(zone, "control_type", f"conflict_zones.{zone_id}")
            polygon = self._require(
                zone, "polygon", f"conflict_zones.{zone_id}"
            )
            self._validate_polygon(
                polygon,
                f"conflict_zones.{zone_id}.polygon",
                required=phase1_ready,
            )
            approaches = self._require(
                zone, "approaches", f"conflict_zones.{zone_id}"
            )
            if not isinstance(approaches, dict):
                raise RoadNetworkError(
                    f"conflict_zones.{zone_id}.approaches must be an object"
                )
            if phase1_ready and not any(
                approach.get("enabled", False)
                for approach in approaches.values()
            ):
                raise RoadNetworkError(
                    f"Phase-1-ready zone {zone_id} has no enabled approaches"
                )
            for approach_id, approach in approaches.items():
                approach_context = (
                    f"conflict_zones.{zone_id}.approaches.{approach_id}"
                )
                for movement_index, movement in enumerate(
                    approach.get("allowed_movements", [])
                ):
                    self._validate_edge(
                        movement,
                        f"{approach_context}.allowed_movements[{movement_index}]",
                    )
                if phase1_ready and approach.get("enabled", False):
                    self._validate_polygon(
                        approach.get("approach_polygon", []),
                        f"{approach_context}.approach_polygon",
                        required=True,
                    )
                    self._validate_line(
                        approach.get("stop_line", []),
                        f"{approach_context}.stop_line",
                        required=True,
                    )
                    self._validate_polygon(
                        approach.get("exit_polygon", []),
                        f"{approach_context}.exit_polygon",
                        required=True,
                    )

    @property
    def unavailable_nodes(self):
        availability = self.data["road_availability"]
        return frozenset(
            availability["disabled_nodes"] + availability["inactive_nodes"]
        )

    def get_route(self, route_id):
        return tuple(self.data["routes"][route_id]["node_sequence"])

    def get_route_config(self, route_id):
        return self.data["routes"][route_id]

    def get_zone(self, zone_id):
        return self.data["conflict_zones"][zone_id]

    @staticmethod
    def point_in_polygon(point, polygon):
        """Return True when an [x, y] point is inside a polygon."""
        x, y = float(point[0]), float(point[1])
        inside = False
        count = len(polygon)
        for index in range(count):
            x1, y1 = polygon[index]
            x2, y2 = polygon[(index + 1) % count]
            if (y1 > y) != (y2 > y):
                crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < crossing_x:
                    inside = not inside
        return inside

    def phase1_ready_zones(self):
        return tuple(
            zone_id
            for zone_id, zone in self.data["conflict_zones"].items()
            if zone.get("enabled", False) and zone.get("phase1_ready", False)
        )


if __name__ == "__main__":
    network = RoadNetwork()
    print(
        f"Validated {network.path.name}: "
        f"{len(network.data['routes'])} routes, "
        f"{len(network.data['conflict_zones'])} conflict zones, "
        f"Phase-1-ready={network.phase1_ready_zones()}"
    )
