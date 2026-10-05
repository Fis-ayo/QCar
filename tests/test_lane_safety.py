import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_safety import (  # noqa: E402
    BoundaryCategorySafetyMonitor,
    LaneSafetyMonitor,
    LaneSafetyReceiver,
)


class LaneSafetyMonitorTests(unittest.TestCase):
    def test_slowdown_requires_debounce(self):
        monitor = LaneSafetyMonitor(slowdown_frames=3)
        self.assertEqual(monitor.update(True, 0.8, -0.5)["state"], "NORMAL")
        self.assertEqual(monitor.update(True, 0.8, -0.5)["state"], "NORMAL")
        result = monitor.update(True, 0.8, -0.5)
        self.assertEqual(result["state"], "SLOWDOWN")
        self.assertEqual(result["speed_gain"], 0.35)

    def test_critical_stop_latches(self):
        monitor = LaneSafetyMonitor(stop_frames=2)
        monitor.update(True, 0.9, -0.3)
        result = monitor.update(True, 0.9, -0.3)
        self.assertEqual(result["state"], "STOP")
        self.assertEqual(result["speed_gain"], 0.0)
        result = monitor.update(True, 0.9, -0.9)
        self.assertEqual(result["state"], "STOP")

    def test_low_confidence_cannot_trigger_intervention(self):
        monitor = LaneSafetyMonitor(slowdown_frames=1, stop_frames=1)
        result = monitor.update(True, 0.2, 0.5)
        self.assertEqual(result["state"], "UNAVAILABLE")
        self.assertEqual(result["speed_gain"], 1.0)

    def test_slowdown_requires_reliable_clear_margin_to_release(self):
        monitor = LaneSafetyMonitor(slowdown_frames=1, clear_frames=2)
        self.assertEqual(
            monitor.update(True, 0.9, -0.5)["state"], "SLOWDOWN"
        )
        self.assertEqual(
            monitor.update(False, 0.0, float("nan"))["state"], "SLOWDOWN"
        )
        monitor.update(True, 0.9, -0.8)
        result = monitor.update(True, 0.9, -0.8)
        self.assertEqual(result["state"], "NORMAL")


class LaneSafetyReceiverTests(unittest.TestCase):
    def test_decode_rejects_invalid_gain(self):
        packet = {
            "version": 1,
            "state": "NORMAL",
            "speed_gain": 2.0,
        }
        with self.assertRaises(ValueError):
            LaneSafetyReceiver.decode(json.dumps(packet).encode("utf-8"))

    def test_stale_transport_fails_closed(self):
        receiver = LaneSafetyReceiver(port=0, stale_after_s=0.5)
        try:
            receiver.last_packet = {
                "version": 1,
                "state": "NORMAL",
                "speed_gain": 1.0,
                "confidence": 0.9,
                "offset": -0.8,
            }
            receiver.last_received_monotonic = 10.0
            result = receiver.receive(now=11.0)
            self.assertEqual(result["state"], "STALE")
            self.assertEqual(result["speed_gain"], 0.0)
        finally:
            receiver.close()


class BoundaryCategorySafetyMonitorTests(unittest.TestCase):
    def test_sustained_inward_risk_slows_then_latches_stop(self):
        monitor = BoundaryCategorySafetyMonitor(
            slowdown_frames=2,
            stop_persistence_s=0.20,
            stop_progress_px=20.0,
        )
        self.assertEqual(
            monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", 151.0, 0.1)["state"],
            "UNAVAILABLE",
        )
        slowed = monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", 160.0, 0.1)
        self.assertEqual(slowed["state"], "SLOWDOWN")
        stopped = monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", 180.0, 0.1)
        self.assertEqual(stopped["state"], "STOP")
        self.assertEqual(stopped["speed_gain"], 0.0)
        self.assertTrue(stopped["stop_latched"])

    def test_missing_data_does_not_release_slowdown(self):
        monitor = BoundaryCategorySafetyMonitor(slowdown_frames=1)
        monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", 151.0, 0.1)
        result = monitor.update("FIXED_ROW_UNAVAILABLE", float("nan"), 0.1)
        self.assertEqual(result["state"], "SLOWDOWN")
        self.assertEqual(result["speed_gain"], 0.25)

    def test_stationary_out_of_range_does_not_stop(self):
        monitor = BoundaryCategorySafetyMonitor(
            slowdown_frames=1,
            stop_persistence_s=0.1,
            stop_progress_px=20.0,
        )
        for _ in range(5):
            result = monitor.update(
                "BEYOND_TOWARD_YELLOW_LIMIT", 155.0, 0.1
            )
        self.assertEqual(result["state"], "SLOWDOWN")

    def test_calibrated_reading_does_not_release_slowdown(self):
        monitor = BoundaryCategorySafetyMonitor(
            slowdown_frames=1,
            clear_frames=2,
        )
        monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", 155.0, 0.1)
        for _ in range(4):
            result = monitor.update("CALIBRATED", 140.0, 0.1)
        self.assertEqual(result["state"], "SLOWDOWN")

        monitor.update("BEYOND_AWAY_LIMIT", 100.0, 0.1)
        result = monitor.update("BEYOND_AWAY_LIMIT", 95.0, 0.1)
        self.assertEqual(result["state"], "NORMAL")

    def test_sustained_calibrated_clearance_releases_slowdown(self):
        monitor = BoundaryCategorySafetyMonitor(
            slowdown_frames=1,
            clear_frames=2,
            recovery_margin_m=0.010,
        )
        monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", 155.0, 0.1)
        monitor.update("CALIBRATED", 60.0, 0.1, -0.012)
        result = monitor.update("CALIBRATED", 58.0, 0.1, -0.013)
        self.assertEqual(result["state"], "NORMAL")

    def test_sustained_retreat_from_peak_suppresses_stale_stop(self):
        monitor = BoundaryCategorySafetyMonitor(
            slowdown_frames=1,
            stop_persistence_s=0.50,
            stop_progress_px=18.0,
            recovery_progress_px=12.0,
            recovery_frames=2,
        )
        for x_value in (154.0, 172.0, 196.0, 184.0, 178.0, 170.0):
            result = monitor.update(
                "BEYOND_TOWARD_YELLOW_LIMIT", x_value, 0.1
            )
        self.assertEqual(result["state"], "SLOWDOWN")
        self.assertGreaterEqual(result["recovery_from_peak_px"], 12.0)
        self.assertGreaterEqual(result["recovery_streak"], 2)

    def test_renewed_inward_motion_restores_stop_eligibility(self):
        monitor = BoundaryCategorySafetyMonitor(
            slowdown_frames=1,
            stop_persistence_s=0.30,
            stop_progress_px=18.0,
            recovery_progress_px=12.0,
            recovery_frames=2,
        )
        for x_value in (154.0, 180.0, 196.0, 180.0, 175.0):
            monitor.update("BEYOND_TOWARD_YELLOW_LIMIT", x_value, 0.1)
        result = monitor.update(
            "BEYOND_TOWARD_YELLOW_LIMIT", 194.0, 0.1
        )
        self.assertEqual(result["state"], "STOP")


if __name__ == "__main__":
    unittest.main()
