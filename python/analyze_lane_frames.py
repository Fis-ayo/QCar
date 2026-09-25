"""Reprocess saved raw front-CSI frames without rerunning a physical QCar."""

import argparse
import csv
import sys
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "qcar"))

from lane_detection import FrontLaneDetector  # noqa: E402


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("frame_directory", type=Path)
parser.add_argument("--output", type=Path, default=Path("lane_offline_analysis"))
args = parser.parse_args()

args.output.mkdir(parents=True, exist_ok=True)
annotated_directory = args.output / "annotated"
annotated_directory.mkdir(exist_ok=True)
detector = FrontLaneDetector()
frames = sorted(args.frame_directory.glob("*.jpg"))
if not frames:
    raise SystemExit(f"No JPG frames found in {args.frame_directory}")

fields = [
    "filename", "yellow_detected", "yellow_confidence", "yellow_near_x_px",
    "yellow_far_x_px", "yellow_offset_normalized", "yellow_heading_rad",
    "yellow_curvature", "yellow_row_coverage", "yellow_fit_rmse_px",
    "yellow_temporal_jump_px", "legacy_two_boundary_detected",
    "yellow_near_y_px", "yellow_far_y_px", "yellow_extrapolated",
    "perspective_calibrated",
]
with (args.output / "lane_offline.csv").open(
    "w", newline="", encoding="utf-8"
) as output_file:
    writer = csv.DictWriter(output_file, fieldnames=fields)
    writer.writeheader()
    for path in frames:
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"Skipping unreadable image: {path}")
            continue
        measurement, mask = detector.process(frame)
        annotated = detector.render(frame, measurement, mask)
        cv2.imwrite(str(annotated_directory / path.name), annotated)
        writer.writerow({
            "filename": path.name,
            "yellow_detected": int(measurement.yellow_detected),
            "yellow_confidence": measurement.yellow_confidence,
            "yellow_near_x_px": measurement.yellow_near_x_px,
            "yellow_far_x_px": measurement.yellow_far_x_px,
            "yellow_offset_normalized": measurement.yellow_offset_normalized,
            "yellow_heading_rad": measurement.yellow_heading_rad,
            "yellow_curvature": measurement.yellow_curvature,
            "yellow_row_coverage": measurement.yellow_row_coverage,
            "yellow_fit_rmse_px": measurement.yellow_fit_rmse_px,
            "yellow_temporal_jump_px": measurement.yellow_temporal_jump_px,
            "yellow_near_y_px": measurement.yellow_near_y_px,
            "yellow_far_y_px": measurement.yellow_far_y_px,
            "yellow_extrapolated": int(measurement.yellow_extrapolated),
            "perspective_calibrated": int(measurement.perspective_calibrated),
            "legacy_two_boundary_detected": int(measurement.detected),
        })

print(
    f"Processed {len(frames)} frame(s). Results: "
    f"{args.output / 'lane_offline.csv'}"
)
