"""Replay the measurement-only yellow-boundary warning on recorded runs."""

import argparse
import sys
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_warning import LaneBoundaryWarning  # noqa: E402


def replay(tracking_path, lane_path):
    tracking = pd.read_csv(tracking_path)
    lane = pd.read_csv(lane_path).sort_values("wall_time_s").copy()
    end_time = float(tracking["wall_time_s"].max())
    # Ignore camera frames recorded after vehicle control was stopped.
    lane = lane[lane["wall_time_s"] <= end_time].copy()

    warning = LaneBoundaryWarning()
    states = []
    for row in lane.itertuples(index=False):
        states.append(
            warning.update(
                row.yellow_detected,
                row.yellow_confidence,
                row.yellow_offset_normalized,
            )
        )
    lane["warning"] = states
    lane["seconds_to_stop"] = end_time - lane["wall_time_s"]

    active = lane[lane["warning"]]
    final = lane[lane["seconds_to_stop"] <= 1.0]
    reference = lane[
        (lane["seconds_to_stop"] >= 3.0)
        & (lane["seconds_to_stop"] <= 6.0)
    ]
    first_lead = (
        float(active["seconds_to_stop"].max()) if not active.empty else None
    )
    return {
        "frames": len(lane),
        "first_warning_lead_s": first_lead,
        "final_coverage": float(final["warning"].mean()) if len(final) else None,
        "reference_coverage": (
            float(reference["warning"].mean()) if len(reference) else None
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "pairs", nargs="+", help="Pairs formatted tracking.csv::lane.csv"
    )
    args = parser.parse_args()

    for pair in args.pairs:
        tracking_name, lane_name = pair.split("::", 1)
        result = replay(tracking_name, lane_name)
        lead = result["first_warning_lead_s"]
        lead_text = "none" if lead is None else f"{lead:.2f} s"
        final = result["final_coverage"]
        reference = result["reference_coverage"]
        final_text = "n/a" if final is None else f"{100 * final:.1f}%"
        reference_text = (
            "n/a" if reference is None else f"{100 * reference:.1f}%"
        )
        print(
            f"{Path(tracking_name).parent.name}: "
            f"frames={result['frames']}, first warning lead={lead_text}, "
            f"final-1s active={final_text}, "
            f"3-6s-earlier active={reference_text}"
        )


if __name__ == "__main__":
    main()
