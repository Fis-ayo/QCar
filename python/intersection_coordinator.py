"""Automatic first-arrival reservation coordinator for Phase 1."""

import argparse
import json
import socket
import time
from collections import deque
from datetime import datetime


VALID_PHASES = {
    "CRUISING",
    "APPROACHING",
    "WAITING",
    "GRANTED",
    "OCCUPYING",
    "EXITED",
}


def log(message):
    timestamp = datetime.now().isoformat(timespec="milliseconds")
    print(f"[{timestamp}] [COORDINATOR] {message}", flush=True)


class ZoneReservation:
    def __init__(self, zone_id, stale_timeout=3.0):
        self.zone_id = zone_id
        self.stale_timeout = stale_timeout
        self.owner = None
        self.owner_phase = None
        self.queue = deque()
        self.queued = set()
        self.clients = {}
        self.blocked_by_stale_owner = False
        self.preferred_car = None

    def _remove_from_queue(self, car_id):
        if car_id not in self.queued:
            return
        self.queue = deque(item for item in self.queue if item != car_id)
        self.queued.discard(car_id)

    def update(self, message, address, now):
        car_id = message["car_id"]
        phase = message["phase"]
        previous = self.clients.get(car_id)
        approach_since = None if previous is None else previous.get("approach_since")
        if phase in {"APPROACHING", "WAITING"}:
            if approach_since is None:
                approach_since = now
        else:
            approach_since = None
        self.clients[car_id] = {
            "last_seen": now,
            "phase": phase,
            "address": address,
            "approach_since": approach_since,
        }

        if phase == "WAITING" and car_id != self.owner:
            if car_id not in self.queued:
                self.queue.append(car_id)
                self.queued.add(car_id)
                log(f"{self.zone_id}: queued {car_id}")
        elif phase in {"CRUISING", "EXITED"} and car_id != self.owner:
            self._remove_from_queue(car_id)

        if car_id == self.owner:
            self.owner_phase = phase
            if phase == "EXITED":
                log(f"{self.zone_id}: released by {car_id}")
                self.preferred_car = self._oldest_other_approach(car_id, now)
                if self.preferred_car is not None:
                    log(
                        f"{self.zone_id}: holding next priority for "
                        f"approaching {self.preferred_car}"
                    )
                self.owner = None
                self.owner_phase = None
                self.blocked_by_stale_owner = False

        self._expire_waiters(now)
        self._check_owner(now)
        self._grant_next(now)

    def _expire_waiters(self, now):
        retained = deque()
        for car_id in self.queue:
            client = self.clients.get(car_id)
            if (
                client is not None
                and now - client["last_seen"] <= self.stale_timeout
                and client["phase"] == "WAITING"
            ):
                retained.append(car_id)
            else:
                self.queued.discard(car_id)
        self.queue = retained

    def _oldest_other_approach(self, previous_owner, now):
        candidates = []
        for car_id, client in self.clients.items():
            if car_id == previous_owner:
                continue
            if (
                now - client["last_seen"] <= self.stale_timeout
                and client["phase"] in {"APPROACHING", "WAITING"}
                and client.get("approach_since") is not None
            ):
                candidates.append((client["approach_since"], car_id))
        if not candidates:
            return None
        return min(candidates)[1]

    def _check_owner(self, now):
        if self.owner is None:
            return
        owner = self.clients.get(self.owner)
        if owner is None or now - owner["last_seen"] > self.stale_timeout:
            if not self.blocked_by_stale_owner:
                log(
                    f"{self.zone_id}: owner {self.owner} is stale; "
                    "zone locked for safety"
                )
            self.blocked_by_stale_owner = True
            return

        # A stale owner keeps ownership while the zone is locked. If that same
        # vehicle resumes sending heartbeats, restore its existing grant. We do
        # not grant another vehicle during the outage, so this cannot create two
        # owners. Previously this lock could only be cleared by EXITED, leaving
        # a vehicle that was still at the stop line permanently deadlocked.
        if self.blocked_by_stale_owner:
            log(
                f"{self.zone_id}: communication restored for owner "
                f"{self.owner}; safety lock cleared"
            )
            self.blocked_by_stale_owner = False

    def _grant_next(self, now):
        if self.owner is not None or self.blocked_by_stale_owner:
            return
        if self.preferred_car is not None:
            preferred = self.clients.get(self.preferred_car)
            if (
                preferred is None
                or now - preferred["last_seen"] > self.stale_timeout
                or preferred["phase"] in {"CRUISING", "EXITED"}
            ):
                self.preferred_car = None
            elif preferred["phase"] == "APPROACHING":
                # The other car is already committed to this approach. Hold
                # the empty zone until it reaches its stop line.
                return
            elif preferred["phase"] == "WAITING":
                car_id = self.preferred_car
                self._remove_from_queue(car_id)
                self.owner = car_id
                self.owner_phase = "WAITING"
                self.preferred_car = None
                log(f"{self.zone_id}: granted to preferred {car_id}")
                return
        while self.queue:
            car_id = self.queue.popleft()
            self.queued.discard(car_id)
            client = self.clients.get(car_id)
            if (
                client is not None
                and now - client["last_seen"] <= self.stale_timeout
                and client["phase"] == "WAITING"
            ):
                self.owner = car_id
                self.owner_phase = "WAITING"
                log(f"{self.zone_id}: granted to {car_id}")
                return

    def response_for(self, car_id):
        return {
            "version": 1,
            "type": "reservation_status",
            "car_id": car_id,
            "zone_id": self.zone_id,
            "granted": car_id == self.owner and not self.blocked_by_stale_owner,
            "owner": self.owner,
            "lease_seconds": self.stale_timeout,
            "zone_blocked": self.blocked_by_stale_owner,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18750)
    parser.add_argument("--zone", default="CZ1_CENTRAL")
    parser.add_argument("--stale-timeout", type=float, default=3.0)
    args = parser.parse_args()

    zone = ZoneReservation(args.zone, stale_timeout=args.stale_timeout)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((args.host, args.port))
    server.settimeout(0.2)
    log(f"Listening on {args.host}:{args.port} for {args.zone}")

    try:
        while True:
            try:
                payload, address = server.recvfrom(4096)
            except socket.timeout:
                now = time.monotonic()
                zone._expire_waiters(now)
                zone._check_owner(now)
                continue

            try:
                message = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if (
                message.get("version") != 1
                or message.get("type") != "vehicle_state"
                or message.get("zone_id") != args.zone
                or message.get("phase") not in VALID_PHASES
                or not isinstance(message.get("car_id"), str)
            ):
                continue

            now = time.monotonic()
            zone.update(message, address, now)
            response = zone.response_for(message["car_id"])
            server.sendto(json.dumps(response).encode("utf-8"), address)
    except KeyboardInterrupt:
        log("Stopped")
    finally:
        server.close()


if __name__ == "__main__":
    main()
