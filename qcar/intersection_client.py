"""QCar-side Phase 1 intersection reservation and stopping logic."""

import json
import socket
import time

import numpy as np


class IntersectionClient:
    """Non-blocking UDP client for the automatic local coordinator."""

    def __init__(
        self,
        coordinator_ip,
        car_id,
        zone_id,
        port=18750,
        send_period=0.05,
        grant_timeout=1.5,
    ):
        self.server = (coordinator_ip, int(port))
        self.car_id = car_id
        self.zone_id = zone_id
        self.send_period = send_period
        self.grant_timeout = grant_timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.last_send = 0.0
        self.last_reply = 0.0
        self.granted = False
        self.owner = None

    def update(self, phase, position, speed):
        now = time.monotonic()
        if now - self.last_send >= self.send_period:
            message = {
                "version": 1,
                "type": "vehicle_state",
                "car_id": self.car_id,
                "zone_id": self.zone_id,
                "phase": phase,
                "position": [float(position[0]), float(position[1])],
                "speed": float(speed),
            }
            try:
                self.socket.sendto(json.dumps(message).encode("utf-8"), self.server)
            except OSError:
                pass
            self.last_send = now

        while True:
            try:
                payload, _ = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                reply = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if (
                reply.get("type") == "reservation_status"
                and reply.get("car_id") == self.car_id
                and reply.get("zone_id") == self.zone_id
            ):
                self.granted = bool(reply.get("granted", False))
                self.owner = reply.get("owner")
                self.last_reply = now

    def has_valid_grant(self):
        return (
            self.granted
            and time.monotonic() - self.last_reply <= self.grant_timeout
        )

    def coordinator_online(self):
        return time.monotonic() - self.last_reply <= self.grant_timeout

    def close(self):
        self.socket.close()


class IntersectionController:
    """Per-vehicle state machine for one stop-controlled conflict zone."""

    CRUISING = "CRUISING"
    APPROACHING = "APPROACHING"
    WAITING = "WAITING"
    GRANTED = "GRANTED"
    OCCUPYING = "OCCUPYING"
    EXITED = "EXITED"

    def __init__(self, road_network, zone_id, approach_id, client, base_speed):
        self.network = road_network
        self.zone_id = zone_id
        self.zone = road_network.get_zone(zone_id)
        self.approach_id = approach_id
        self.approach = self.zone["approaches"][approach_id]
        self.client = client
        self.base_speed = float(base_speed)
        self.phase = self.CRUISING
        self.stop_required = False
        self.stopped_since = None
        self.exited_since = None
        self.last_reported_phase = None
        self.distance_to_stop = float("inf")
        self.in_conflict = False
        self.in_exit = False

        stopping = self.zone["stopping"]
        self.approach_speed = float(stopping["approach_speed_mps"])
        self.minimum_crawl_speed = float(
            stopping.get("minimum_crawl_speed_mps", 0.08)
        )
        self.stopped_speed = float(stopping["stopped_speed_threshold_mps"])
        self.minimum_dwell = float(stopping["minimum_dwell_seconds"])
        self.stop_tolerance = float(
            stopping.get("stop_capture_distance_m", 0.15)
        )
        self.slowdown_distance = float(
            stopping.get("slowdown_distance_m", 0.50)
        )
        self.exit_hold_seconds = float(
            stopping.get("exit_hold_seconds", 0.75)
        )

    @staticmethod
    def _distance_to_line(point, line):
        point = np.asarray(point, dtype=float)
        start = np.asarray(line[0], dtype=float)
        end = np.asarray(line[1], dtype=float)
        segment = end - start
        length_squared = float(np.dot(segment, segment))
        if length_squared == 0:
            return float(np.linalg.norm(point - start))
        projection = np.clip(
            np.dot(point - start, segment) / length_squared,
            0.0,
            1.0,
        )
        closest = start + projection * segment
        return float(np.linalg.norm(point - closest))

    def _inside(self, point, polygon):
        return self.network.point_in_polygon(point, polygon)

    def _set_phase(self, phase):
        if phase != self.phase:
            print(
                f"[INTERSECTION] {self.zone_id} {self.phase} -> {phase}",
                flush=True,
            )
            self.phase = phase

    def update(self, position, speed):
        now = time.monotonic()
        in_approach = self._inside(
            position, self.approach["approach_polygon"]
        )
        in_conflict = self._inside(position, self.zone["polygon"])
        in_exit = self._inside(position, self.approach["exit_polygon"])
        distance_to_stop = self._distance_to_line(
            position, self.approach["stop_line"]
        )
        self.distance_to_stop = distance_to_stop
        self.in_conflict = in_conflict
        self.in_exit = in_exit

        if self.phase == self.CRUISING and in_approach:
            self.stop_required = False
            self.stopped_since = None
            self._set_phase(self.APPROACHING)

        if self.phase == self.APPROACHING:
            if distance_to_stop <= self.stop_tolerance:
                self.stop_required = True
            if self.stop_required and abs(speed) <= self.stopped_speed:
                if self.stopped_since is None:
                    self.stopped_since = now
                self._set_phase(self.WAITING)

        if self.phase == self.WAITING:
            dwell_complete = (
                self.stopped_since is not None
                and now - self.stopped_since >= self.minimum_dwell
            )
            if dwell_complete and self.client.has_valid_grant():
                self._set_phase(self.GRANTED)

        if self.phase == self.GRANTED:
            if in_conflict:
                self._set_phase(self.OCCUPYING)
            elif not self.client.has_valid_grant():
                self._set_phase(self.WAITING)

        if self.phase == self.OCCUPYING and in_exit:
            self.exited_since = now
            self._set_phase(self.EXITED)

        if self.phase == self.EXITED:
            if self.exited_since is None:
                self.exited_since = now
            if now - self.exited_since >= self.exit_hold_seconds and not in_conflict:
                self.stop_required = False
                self.stopped_since = None
                self._set_phase(self.CRUISING)

        self.client.update(self.phase, position, speed)

        if self.phase == self.APPROACHING:
            if self.stop_required:
                return 0.0
            span = max(self.slowdown_distance - self.stop_tolerance, 0.01)
            ratio = np.clip(
                (distance_to_stop - self.stop_tolerance) / span,
                0.0,
                1.0,
            )
            target_speed = min(
                self.approach_speed,
                max(self.minimum_crawl_speed, self.approach_speed * ratio),
            )
            return target_speed / self.base_speed

        if self.phase == self.WAITING:
            return 0.0

        # Once inside the conflict zone, continue clearing it even if
        # communication is lost. Before entry, GRANTED requires a live lease.
        if self.phase == self.GRANTED and not self.client.has_valid_grant():
            return 0.0

        return 1.0

    def close(self):
        self.client.close()
