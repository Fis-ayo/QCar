import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_safety import LaneSafetyMonitor, LaneSafetyReceiver  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
