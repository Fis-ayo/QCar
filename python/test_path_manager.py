"""Tests for general generated-path validation and repair."""

import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from hal.products.mats import SDCSRoadMap  # noqa: E402
from path_manager import PathManager  # noqa: E402
from road_rules import RoadNetwork  # noqa: E402


class PathManagerTests(unittest.TestCase):
    def setUp(self):
        self.roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.network = RoadNetwork(ROOT / "qcar" / "road_network.json")

    def manager_for(self, route_id):
        nodes = self.network.get_route(route_id)
        waypoints = self.roadmap.generate_path(list(nodes))
        return PathManager(waypoints, nodes, self.roadmap)

    def test_node_16_reversal_is_detected_and_repaired(self):
        manager = self.manager_for("START_NODE_11")
        raw_errors, _ = manager.validate(manager.original_waypoints)
        reversal_errors = [
            error for error in raw_errors if "Heading reversal" in error
        ]
        self.assertEqual(len(reversal_errors), 2)

        result = manager.prepare()
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(len(result.repairs), 1)
        self.assertEqual(result.repairs[0]["removed_index"], 804)
        self.assertEqual(
            result.waypoints.shape[1],
            manager.original_waypoints.shape[1] - 1,
        )

    def test_repair_preserves_every_route_node(self):
        manager = self.manager_for("START_NODE_11")
        result = manager.prepare()
        self.assertTrue(result.valid, result.errors)
        for node in manager.node_sequence:
            pose = self.roadmap.get_node_pose(node).squeeze()[:2]
            distance = np.min(
                np.linalg.norm(result.waypoints - pose[:, None], axis=0)
            )
            self.assertLessEqual(distance, manager.node_tolerance)

    def test_node_0_route_requires_no_repair(self):
        manager = self.manager_for("START_NODE_0")
        result = manager.prepare()
        self.assertTrue(result.valid, result.errors)
        self.assertEqual(result.repairs, [])
        np.testing.assert_array_equal(
            result.waypoints,
            manager.original_waypoints,
        )

    def test_non_finite_path_fails_closed(self):
        manager = self.manager_for("START_NODE_0")
        manager.original_waypoints[0, 50] = np.nan
        result = manager.prepare()
        self.assertFalse(result.valid)
        self.assertTrue(
            any("non-finite" in error for error in result.errors)
        )

    def test_unrepairable_anchor_reversal_fails_closed(self):
        nodes = self.network.get_route("START_NODE_0")
        anchor = self.roadmap.get_node_pose(nodes[0]).squeeze()[:2]
        malformed = np.column_stack([
            anchor,
            anchor + [0.01, 0.0],
            anchor,
            anchor + [0.01, 0.0],
        ])
        manager = PathManager(malformed, nodes, self.roadmap)
        result = manager.prepare()
        self.assertFalse(result.valid)


if __name__ == "__main__":
    unittest.main()
