"""Confidence-gated CSI lane safety states and local UDP transport."""

import json
import math
import socket
import time


LANE_SAFETY_PORT = 18760


class LaneSafetyMonitor:
    """Convert yellow-boundary measurements into bounded speed restrictions.

    STOP is deliberately latched. This layer never produces steering commands.
    Thresholds are normalized image-space values derived from the existing
    .151 datasets and must remain experimental until physical A/B validation.
    """

    NORMAL = "NORMAL"
    SLOWDOWN = "SLOWDOWN"
    STOP = "STOP"
    UNAVAILABLE = "UNAVAILABLE"

    def __init__(
        self,
        minimum_confidence=0.60,
        slowdown_offset=-0.60,
        slowdown_exit_offset=-0.70,
        stop_offset=-0.40,
        slowdown_gain=0.35,
        slowdown_frames=4,
        stop_frames=3,
        clear_frames=8,
    ):
        self.minimum_confidence = float(minimum_confidence)
        self.slowdown_offset = float(slowdown_offset)
        self.slowdown_exit_offset = float(slowdown_exit_offset)
        self.stop_offset = float(stop_offset)
        self.slowdown_gain = float(slowdown_gain)
        self.slowdown_frames = int(slowdown_frames)
        self.stop_frames = int(stop_frames)
        self.clear_frames = int(clear_frames)
        self.state = self.UNAVAILABLE
        self.slowdown_streak = 0
        self.stop_streak = 0
        self.clear_streak = 0
        self.stop_latched = False

    @property
    def speed_gain(self):
        if self.state == self.STOP:
            return 0.0
        if self.state == self.SLOWDOWN:
            return self.slowdown_gain
        return 1.0

    def update(self, detected, confidence, offset):
        confidence = float(confidence)
        try:
            offset = float(offset)
        except (TypeError, ValueError):
            offset = float("nan")
        reliable = (
            bool(detected)
            and math.isfinite(confidence)
            and confidence >= self.minimum_confidence
            and math.isfinite(offset)
        )

        if self.stop_latched:
            self.state = self.STOP
            return self.snapshot(reliable, confidence, offset)

        stop_near = reliable and offset >= self.stop_offset
        slowdown_near = reliable and offset >= self.slowdown_offset
        clear = reliable and offset <= self.slowdown_exit_offset

        self.stop_streak = self.stop_streak + 1 if stop_near else 0
        if self.stop_streak >= self.stop_frames:
            self.stop_latched = True
            self.state = self.STOP
            self.slowdown_streak = 0
            self.clear_streak = 0
            return self.snapshot(reliable, confidence, offset)

        if self.state != self.SLOWDOWN:
            self.slowdown_streak = (
                self.slowdown_streak + 1 if slowdown_near else 0
            )
            self.clear_streak = 0
            if self.slowdown_streak >= self.slowdown_frames:
                self.state = self.SLOWDOWN
                self.slowdown_streak = 0
            elif reliable:
                self.state = self.NORMAL
            else:
                self.state = self.UNAVAILABLE
        else:
            # Missing/weak measurements hold slowdown briefly rather than
            # immediately releasing it. A reliable clear margin is required.
            self.clear_streak = self.clear_streak + 1 if clear else 0
            if self.clear_streak >= self.clear_frames:
                self.state = self.NORMAL
                self.clear_streak = 0

        return self.snapshot(reliable, confidence, offset)

    def snapshot(self, reliable, confidence, offset):
        return {
            "state": self.state,
            "speed_gain": self.speed_gain,
            "reliable": bool(reliable),
            "confidence": float(confidence),
            "offset": float(offset),
            "stop_latched": bool(self.stop_latched),
            "slowdown_streak": self.slowdown_streak,
            "stop_streak": self.stop_streak,
            "clear_streak": self.clear_streak,
        }


class LaneSafetyPublisher:
    def __init__(self, host="127.0.0.1", port=LANE_SAFETY_PORT, car_id=""):
        self.address = (host, int(port))
        self.car_id = str(car_id)
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, decision):
        self.sequence += 1
        packet = dict(decision)
        packet.update({
            "version": 1,
            "car_id": self.car_id,
            "sequence": self.sequence,
            "sent_wall_time_s": time.time(),
        })
        self.socket.sendto(
            json.dumps(packet, separators=(",", ":")).encode("utf-8"),
            self.address,
        )

    def close(self):
        self.socket.close()


class LaneSafetyReceiver:
    """Receive the latest local lane decision; stale transport fails closed."""

    def __init__(
        self,
        host="127.0.0.1",
        port=LANE_SAFETY_PORT,
        expected_car_id=None,
        stale_after_s=0.50,
    ):
        self.expected_car_id = expected_car_id
        self.stale_after_s = float(stale_after_s)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)
        self.last_packet = None
        self.last_received_monotonic = None

    @staticmethod
    def decode(data):
        packet = json.loads(data.decode("utf-8"))
        if packet.get("version") != 1:
            raise ValueError("Unsupported lane-safety packet version")
        if packet.get("state") not in {
            LaneSafetyMonitor.NORMAL,
            LaneSafetyMonitor.SLOWDOWN,
            LaneSafetyMonitor.STOP,
            LaneSafetyMonitor.UNAVAILABLE,
        }:
            raise ValueError("Invalid lane-safety state")
        gain = float(packet["speed_gain"])
        if not 0.0 <= gain <= 1.0:
            raise ValueError("Invalid lane-safety speed gain")
        return packet

    def receive(self, now=None):
        now = time.monotonic() if now is None else float(now)
        while True:
            try:
                data, _ = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                packet = self.decode(data)
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                continue
            if (
                self.expected_car_id
                and packet.get("car_id") != self.expected_car_id
            ):
                continue
            self.last_packet = packet
            self.last_received_monotonic = now

        if (
            self.last_packet is None
            or self.last_received_monotonic is None
            or now - self.last_received_monotonic > self.stale_after_s
        ):
            return {
                "state": "STALE",
                "speed_gain": 0.0,
                "reliable": False,
                "confidence": float("nan"),
                "offset": float("nan"),
                "stop_latched": False,
                "age_s": float("inf") if self.last_received_monotonic is None
                else now - self.last_received_monotonic,
            }

        result = dict(self.last_packet)
        result["age_s"] = now - self.last_received_monotonic
        return result

    def close(self):
        self.socket.close()
