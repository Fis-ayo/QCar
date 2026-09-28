"""Replay the current lane slowdown/stop logic on recorded CSI data."""

import argparse
import sys
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_safety import LaneSafetyMonitor  # noqa: E402


def replay(tracking_path, lane_path):
    tracking = pd.read_csv(tracking_path)
    lane = pd.read_csv(lane_path).sort_values("wall_time_s")
    end_time = float(tracking["wall_time_s"].max())
    lane = lane[lane["wall_time_s"] <= end_time]
    monitor = LaneSafetyMonitor()
    first_lead = {}
    for row in lane.itertuples(index=False):
        state = monitor.update(
            row.yellow_detected,
            row.yellow_confidence,
            row.yellow_offset_normalized,
        )["state"]
        first_lead.setdefault(state, end_time - float(row.wall_time_s))
    return first_lead


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "pairs", nargs="+", help="Pairs formatted tracking.csv::lane.csv"
    )
    args = parser.parse_args()
    for pair in args.pairs:
        tracking_path, lane_path = pair.split("::", 1)
        first = replay(tracking_path, lane_path)
        slowdown = first.get("SLOWDOWN")
        stop = first.get("STOP")
        print(
            f"{Path(tracking_path).parent.name}: "
            f"slowdown lead={'none' if slowdown is None else f'{slowdown:.2f} s'}, "
            f"stop lead={'none' if stop is None else f'{stop:.2f} s'}"
        )


if __name__ == "__main__":
    main()
