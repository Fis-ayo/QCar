"""Local tests for the Phase 1 coordinator and vehicle state machine."""

import sys
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))
sys.path.insert(0, str(ROOT / "python"))

from intersection_client import IntersectionController  # noqa: E402
from intersection_coordinator import ZoneReservation  # noqa: E402
from road_rules import RoadNetwork  # noqa: E402


class FakeClient:
    def __init__(self):
        self.granted = False
        self.messages = []

    def update(self, phase, position, speed):
        self.messages.append((phase, tuple(position), speed))

    def has_valid_grant(self):
        return self.granted

    def close(self):
        pass


class IntersectionControllerTests(unittest.TestCase):
    def setUp(self):
        self.network = RoadNetwork(ROOT / "qcar" / "road_network.json")
        self.client = FakeClient()
        self.controller = IntersectionController(
            road_network=self.network,
            zone_id="CZ1_CENTRAL",
            approach_id="FROM_NORTH_NODE_12",
            client=self.client,
            base_speed=0.25,
        )

    def test_stop_grant_occupy_exit_cycle(self):
        gain = self.controller.update([0.0, 2.0], 0.25)
        self.assertEqual(self.controller.phase, self.controller.APPROACHING)
        self.assertLess(gain, 1.0)
        self.assertGreaterEqual(
            gain,
            self.controller.minimum_crawl_speed / self.controller.base_speed,
        )

        gain = self.controller.update([0.0, 1.77], 0.10)
        self.assertEqual(gain, 0.0)
        self.controller.update([0.0, 1.77], 0.0)
        self.assertEqual(self.controller.phase, self.controller.WAITING)

        self.controller.stopped_since = time.monotonic() - 2.0
        self.client.granted = True
        self.controller.update([0.0, 1.77], 0.0)
        self.assertEqual(self.controller.phase, self.controller.GRANTED)

        self.controller.update([0.40, 0.96], 0.15)
        self.assertEqual(self.controller.phase, self.controller.OCCUPYING)

        self.client.granted = False
        gain = self.controller.update([1.60, 0.90], 0.15)
        self.assertEqual(self.controller.phase, self.controller.EXITED)
        self.assertEqual(gain, 1.0)

    def test_lost_grant_before_entry_returns_to_waiting(self):
        self.controller.phase = self.controller.GRANTED
        self.client.granted = False
        gain = self.controller.update([0.0, 1.60], 0.05)
        self.assertEqual(self.controller.phase, self.controller.WAITING)
        self.assertEqual(gain, 0.0)

    def test_communication_loss_while_occupying_keeps_clearing(self):
        self.controller.phase = self.controller.OCCUPYING
        self.client.granted = False
        gain = self.controller.update([0.40, 0.96], 0.15)
        self.assertEqual(self.controller.phase, self.controller.OCCUPYING)
        self.assertEqual(gain, 1.0)


class CoordinatorTests(unittest.TestCase):
    @staticmethod
    def message(car_id, phase):
        return {
            "car_id": car_id,
            "phase": phase,
        }

    def test_first_arrival_and_release(self):
        zone = ZoneReservation("CZ1_CENTRAL")
        now = time.monotonic()
        zone.update(self.message("car-a", "WAITING"), ("127.0.0.1", 1), now)
        zone.update(self.message("car-b", "WAITING"), ("127.0.0.1", 2), now + 0.01)
        self.assertEqual(zone.owner, "car-a")
        self.assertTrue(zone.response_for("car-a")["granted"])
        self.assertFalse(zone.response_for("car-b")["granted"])

        zone.update(self.message("car-a", "EXITED"), ("127.0.0.1", 1), now + 0.02)
        self.assertEqual(zone.owner, "car-b")
        self.assertTrue(zone.response_for("car-b")["granted"])

    def test_approaching_other_car_prevents_repeat_grant(self):
        zone = ZoneReservation("CZ1_CENTRAL")
        now = time.monotonic()
        zone.update(self.message("car-a", "WAITING"), ("127.0.0.1", 1), now)
        zone.update(
            self.message("car-b", "APPROACHING"),
            ("127.0.0.1", 2),
            now + 0.01,
        )
        zone.update(
            self.message("car-a", "EXITED"),
            ("127.0.0.1", 1),
            now + 0.02,
        )
        zone.update(
            self.message("car-a", "WAITING"),
            ("127.0.0.1", 1),
            now + 0.03,
        )
        self.assertIsNone(zone.owner)
        self.assertEqual(zone.preferred_car, "car-b")

        zone.update(
            self.message("car-b", "WAITING"),
            ("127.0.0.1", 2),
            now + 0.04,
        )
        self.assertEqual(zone.owner, "car-b")

    def test_same_owner_heartbeat_clears_stale_safety_lock(self):
        zone = ZoneReservation("CZ1_CENTRAL", stale_timeout=3.0)
        now = time.monotonic()
        zone.update(self.message("car-a", "WAITING"), ("127.0.0.1", 1), now)
        zone.update(
            self.message("car-b", "WAITING"),
            ("127.0.0.1", 2),
            now + 0.01,
        )

        zone._check_owner(now + 3.1)
        self.assertTrue(zone.blocked_by_stale_owner)
        self.assertFalse(zone.response_for("car-a")["granted"])
        self.assertFalse(zone.response_for("car-b")["granted"])

        zone.update(
            self.message("car-a", "WAITING"),
            ("127.0.0.1", 1),
            now + 3.2,
        )
        self.assertFalse(zone.blocked_by_stale_owner)
        self.assertEqual(zone.owner, "car-a")
        self.assertTrue(zone.response_for("car-a")["granted"])
        self.assertFalse(zone.response_for("car-b")["granted"])


if __name__ == "__main__":
    unittest.main()
