"""Validate and conservatively repair generated SDCS waypoint paths."""

from dataclasses import dataclass, field

import numpy as np

from pal.utilities.math import wrap_to_pi


@dataclass
class PathPreparationResult:
    valid: bool
    waypoints: np.ndarray
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    repairs: list = field(default_factory=list)


class PathManager:
    """Prepare a route without embedding rules for particular node numbers."""

    def __init__(
        self,
        waypoints,
        node_sequence,
        roadmap,
        reversal_threshold_degrees=150.0,
        maximum_gap_m=0.05,
        closure_tolerance_m=0.05,
        node_tolerance_m=0.02,
        anchor_protection_tolerance_m=1e-4,
        zero_length_tolerance_m=1e-7,
    ):
        self.original_waypoints = np.asarray(waypoints, dtype=float)
        self.node_sequence = tuple(int(node) for node in node_sequence)
        self.roadmap = roadmap
        self.reversal_threshold = np.deg2rad(reversal_threshold_degrees)
        self.maximum_gap = float(maximum_gap_m)
        self.closure_tolerance = float(closure_tolerance_m)
        self.node_tolerance = float(node_tolerance_m)
        self.anchor_protection_tolerance = float(
            anchor_protection_tolerance_m
        )
        self.zero_length_tolerance = float(zero_length_tolerance_m)
        self.anchor_positions = self._node_positions()

    def _node_positions(self):
        positions = []
        for node in self.node_sequence:
            pose = np.asarray(self.roadmap.get_node_pose(node)).squeeze()
            positions.append(np.asarray(pose[:2], dtype=float))
        return positions

    @staticmethod
    def _geometry(waypoints):
        segments = np.diff(waypoints, axis=1)
        lengths = np.linalg.norm(segments, axis=0)
        headings = np.arctan2(segments[1], segments[0])
        turns = np.asarray([
            wrap_to_pi(headings[index] - headings[index - 1])
            for index in range(1, len(headings))
        ])
        return segments, lengths, headings, turns

    def _is_anchor(self, point):
        return any(
            np.linalg.norm(point - anchor) <= self.anchor_protection_tolerance
            for anchor in self.anchor_positions
        )

    def validate(self, waypoints):
        waypoints = np.asarray(waypoints, dtype=float)
        errors = []
        warnings = []

        if waypoints.ndim != 2 or waypoints.shape[0] != 2:
            return [
                "Waypoints must be a finite 2xN array of [x, y] points."
            ], warnings
        if waypoints.shape[1] < 3:
            return ["A route requires at least three waypoints."], warnings
        if not np.all(np.isfinite(waypoints)):
            errors.append("Waypoint path contains non-finite coordinates.")
            return errors, warnings

        _, lengths, _, turns = self._geometry(waypoints)
        zero_segments = np.flatnonzero(
            lengths <= self.zero_length_tolerance
        )
        for index in zero_segments:
            errors.append(f"Zero-length segment at waypoint {index}.")

        long_segments = np.flatnonzero(lengths > self.maximum_gap)
        for index in long_segments:
            errors.append(
                f"Waypoint gap {index}->{index + 1} is "
                f"{lengths[index]:.4f} m; maximum is "
                f"{self.maximum_gap:.4f} m."
            )

        reversals = np.flatnonzero(np.abs(turns) >= self.reversal_threshold)
        for turn_index in reversals:
            vertex_index = int(turn_index + 1)
            errors.append(
                f"Heading reversal of {np.degrees(turns[turn_index]):.2f} "
                f"degrees at waypoint {vertex_index}."
            )

        closure_error = float(
            np.linalg.norm(waypoints[:, 0] - waypoints[:, -1])
        )
        if self.node_sequence[0] == self.node_sequence[-1]:
            if closure_error > self.closure_tolerance:
                errors.append(
                    f"Closed route endpoints differ by {closure_error:.4f} m."
                )

        for node, anchor in zip(self.node_sequence, self.anchor_positions):
            nearest_distance = float(
                np.min(np.linalg.norm(waypoints - anchor[:, None], axis=0))
            )
            if nearest_distance > self.node_tolerance:
                errors.append(
                    f"Route node {node} is {nearest_distance:.4f} m from "
                    "the nearest waypoint."
                )

        short_segments = np.flatnonzero(
            (lengths > self.zero_length_tolerance) & (lengths < 0.002)
        )
        for index in short_segments:
            warnings.append(
                f"Very short segment {index}->{index + 1}: "
                f"{lengths[index]:.4f} m."
            )
        return errors, warnings

    def _paired_reversals(self, waypoints):
        _, _, _, turns = self._geometry(waypoints)
        pairs = []
        for index in range(len(turns) - 1):
            first = turns[index]
            second = turns[index + 1]
            if (
                abs(first) >= self.reversal_threshold
                and abs(second) >= self.reversal_threshold
                and np.sign(first) != np.sign(second)
            ):
                # turns[index] is at point index+1; the next turn is at
                # point index+2. Either point can form the spike, subject to
                # preservation of route-node anchors.
                pairs.append((index + 1, index + 2, first, second))
        return pairs

    @staticmethod
    def _local_max_turn(waypoints, removed_index):
        candidate = np.delete(waypoints, removed_index, axis=1)
        _, _, _, turns = PathManager._geometry(candidate)
        if turns.size == 0:
            return 0.0
        start = max(0, removed_index - 3)
        stop = min(len(turns), removed_index + 2)
        return float(np.max(np.abs(turns[start:stop])))

    def _repair_one_paired_reversal(self, waypoints):
        pairs = self._paired_reversals(waypoints)
        if not pairs:
            return waypoints, None

        first_index, second_index, first_turn, second_turn = pairs[0]
        candidates = []
        for index in (first_index, second_index):
            point = waypoints[:, index]
            if self._is_anchor(point):
                continue
            candidates.append((self._local_max_turn(waypoints, index), index))

        if not candidates:
            return waypoints, None

        _, removed_index = min(candidates)
        removed_point = waypoints[:, removed_index].copy()
        repaired = np.delete(waypoints, removed_index, axis=1)
        repair = {
            "type": "paired_heading_reversal",
            "removed_index": int(removed_index),
            "removed_point": removed_point.tolist(),
            "first_turn_degrees": float(np.degrees(first_turn)),
            "second_turn_degrees": float(np.degrees(second_turn)),
        }
        return repaired, repair

    def prepare(self):
        waypoints = self.original_waypoints.copy()
        repairs = []

        # Only paired, opposite, near-180-degree spikes are repaired
        # automatically. Other invalid geometry remains fail-closed.
        for _ in range(10):
            repaired, repair = self._repair_one_paired_reversal(waypoints)
            if repair is None:
                break
            waypoints = repaired
            repairs.append(repair)

        errors, warnings = self.validate(waypoints)
        return PathPreparationResult(
            valid=not errors,
            waypoints=waypoints,
            errors=errors,
            warnings=warnings,
            repairs=repairs,
        )
