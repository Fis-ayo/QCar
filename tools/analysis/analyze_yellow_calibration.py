"""Reprocess stationary calibration frames at a fixed image row."""

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "qcar"))

from lane_detection import FrontLaneDetector  # noqa: E402


def summarize(values):
    return {
        "count": len(values),
        "mean": statistics.fmean(values) if values else math.nan,
        "median": statistics.median(values) if values else math.nan,
        "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "minimum": min(values) if values else math.nan,
        "maximum": max(values) if values else math.nan,
    }


parser = argparse.ArgumentParser()
parser.add_argument("calibration_directory", type=Path)
parser.add_argument(
    "--output", type=Path, default=None,
    help="Defaults to fixed_row_analysis.json inside the calibration folder.",
)
args = parser.parse_args()

calibration_directory = args.calibration_directory.resolve()
output_path = args.output or calibration_directory / "fixed_row_analysis.json"
datasets = []

for csv_path in sorted(calibration_directory.glob("*.csv")):
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        continue
    label = rows[0]
    distance_text = label.get("vehicle_center_to_yellow_mm", "")
    if not distance_text:
        distance_text = label.get("yellow_clearance_mm", "")

    frame_directory = calibration_directory / (
        csv_path.stem + "_frames"
    ) / "raw"
    detector = FrontLaneDetector()
    measurements = []
    for image_path in sorted(frame_directory.glob("*.jpg")):
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue
        measurement, _ = detector.process(frame)
        if measurement.yellow_reference_detected:
            measurements.append({
                "file": image_path.name,
                "x_px": measurement.yellow_reference_x_px,
                "y_px": measurement.yellow_reference_y_px,
                "confidence": measurement.yellow_reference_confidence,
            })

    x_values = [item["x_px"] for item in measurements]
    datasets.append({
        "csv": csv_path.name,
        "pose": label.get("calibration_pose", ""),
        "road_section": label.get("road_section", ""),
        "vehicle_center_to_yellow_mm": float(distance_text),
        "available_frames": len(list(frame_directory.glob("*.jpg"))),
        "detected_frames": len(measurements),
        "fixed_row_x_px": summarize(x_values),
        "measurements": measurements,
    })

def compare_datasets(group):
    if len(group) < 2:
        return {
            "status": "insufficient_poses",
            "required": 2,
            "available": len(group),
        }
    ordered = sorted(
        group, key=lambda item: item["vehicle_center_to_yellow_mm"]
    )
    pairwise = []
    for first, second in zip(ordered, ordered[1:]):
        distance_change = (
            second["vehicle_center_to_yellow_mm"]
            - first["vehicle_center_to_yellow_mm"]
        )
        pixel_change = (
            second["fixed_row_x_px"]["mean"]
            - first["fixed_row_x_px"]["mean"]
        )
        pooled_noise = math.sqrt(
            first["fixed_row_x_px"]["stdev"] ** 2
            + second["fixed_row_x_px"]["stdev"] ** 2
        )
        pairwise.append({
            "from_pose": first["pose"],
            "to_pose": second["pose"],
            "distance_change_mm": distance_change,
            "fixed_row_pixel_change_px": pixel_change,
            "absolute_signal_to_noise": (
                abs(pixel_change) / pooled_noise
                if pooled_noise > 0 else math.inf
            ),
            "pixels_per_mm": (
                pixel_change / distance_change
                if distance_change != 0 else math.nan
            ),
        })

    distances = [item["vehicle_center_to_yellow_mm"] for item in ordered]
    pixels = [item["fixed_row_x_px"]["mean"] for item in ordered]
    mean_distance = statistics.fmean(distances)
    mean_pixel = statistics.fmean(pixels)
    denominator = sum((item - mean_distance) ** 2 for item in distances)
    slope = sum(
        (distance - mean_distance) * (pixel - mean_pixel)
        for distance, pixel in zip(distances, pixels)
    ) / denominator
    intercept = mean_pixel - slope * mean_distance
    predictions = [intercept + slope * item for item in distances]
    residuals = [
        actual - predicted for actual, predicted in zip(pixels, predictions)
    ]
    total_variation = sum((item - mean_pixel) ** 2 for item in pixels)
    residual_variation = sum(item ** 2 for item in residuals)
    return {
        "status": "evaluated",
        "monotonic": all(
            pixels[index] > pixels[index + 1]
            for index in range(len(pixels) - 1)
        ),
        "pairwise": pairwise,
        "linear_fit": {
            "slope_pixels_per_mm": slope,
            "intercept_pixels": intercept,
            "r_squared": (
                1.0 - residual_variation / total_variation
                if total_variation > 0 else math.nan
            ),
            "maximum_absolute_residual_px": max(abs(item) for item in residuals),
        },
    }


sections = sorted({item["road_section"] for item in datasets})
comparisons_by_section = {
    section: compare_datasets([
        item for item in datasets if item["road_section"] == section
    ])
    for section in sections
}

report = {
    "method": "observed yellow pixels at 85% of detector ROI height",
    "datasets": datasets,
    "comparisons_by_section": comparisons_by_section,
}
output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

for dataset in datasets:
    stats = dataset["fixed_row_x_px"]
    print(
        f"{dataset['pose']}: {dataset['detected_frames']}/"
        f"{dataset['available_frames']} frames, x={stats['mean']:.2f} "
        f"+/- {stats['stdev']:.2f} px"
    )
for section, comparison in comparisons_by_section.items():
    if comparison["status"] != "evaluated":
        print(
            f"{section}: comparison needs at least "
            f"{comparison['required']} poses; has {comparison['available']}"
        )
        continue
    print(f"{section} monotonic: {comparison['monotonic']}")
    for pair in comparison["pairwise"]:
        print(
            f"{pair['from_pose']} -> {pair['to_pose']}: "
            f"{pair['distance_change_mm']:.1f} mm, "
            f"{pair['fixed_row_pixel_change_px']:.2f} px, "
            f"{pair['pixels_per_mm']:.2f} px/mm"
        )
    fit = comparison["linear_fit"]
    print(
        f"Linear fit: R^2={fit['r_squared']:.3f}, "
        f"max residual={fit['maximum_absolute_residual_px']:.2f} px"
    )
print(f"Report: {output_path}")
