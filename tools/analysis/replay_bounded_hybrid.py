"""Replay the active bounded hybrid fusion against recorded tracking CSV."""

import argparse
import csv
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "qcar"))

from lane_guidance import BoundedHybridSteering, HybridLaneAdvisor  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("tracking_csv", type=Path)
    args = parser.parse_args()
    fusion = BoundedHybridSteering()
    previous_time = None
    corrections = []
    risk_corrections = []
    with args.tracking_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            elapsed = float(row["elapsed_s"])
            dt = 0.0 if previous_time is None else elapsed - previous_time
            previous_time = elapsed
            category = row["lane_guidance_boundary_category"]
            candidate = float(row["lane_guidance_candidate_rad"])
            calibrated = row["lane_guidance_calibration_reason"] == "CALIBRATED"
            guidance = {
                "state": HybridLaneAdvisor.ACTIVE if calibrated else "UNAVAILABLE",
                "confidence": float(row["lane_guidance_confidence"]),
                "boundary_category": category,
                "applied_correction_rad": candidate if calibrated else 0.0,
            }
            result = fusion.update(
                float(row["waypoint_steering_command_rad"]), guidance, dt
            )
            correction = result["protective_correction_rad"]
            corrections.append(correction)
            if category == "BEYOND_TOWARD_YELLOW_LIMIT":
                risk_corrections.append(correction)
    away = sum(value < 0.0 for value in risk_corrections)
    print(f"rows={len(corrections)}")
    print(f"maximum_abs_correction_rad={max(map(abs, corrections)):.6f}")
    print(f"risk_rows={len(risk_corrections)}")
    print(f"risk_rows_correcting_away={away}")
    print(f"risk_rows_wrong_direction={sum(value > 0.0 for value in risk_corrections)}")


if __name__ == "__main__":
    main()
