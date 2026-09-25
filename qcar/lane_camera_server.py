"""Run measurement-only lane diagnostics using the QCar2 front CSI camera."""

import argparse
import csv
import os
import time

import cv2
from pal.products.qcar import QCarCameras
from pal.utilities.probe import Probe

from lane_detection import FrontLaneDetector
from lane_warning import LaneBoundaryWarning


parser = argparse.ArgumentParser(prog="Front CSI lane diagnostics")
parser.add_argument("--car_id", required=True)
parser.add_argument("--ip_host", default="192.168.2.10")
parser.add_argument("--probing", action="store_true")
parser.add_argument("--width", type=int, default=1640)
parser.add_argument("--height", type=int, default=820)
parser.add_argument("--frame_rate", type=float, default=30.0)
parser.add_argument("--csv_path", default="lane_diagnostics.csv")
parser.add_argument("--snapshot_directory", default="lane_frames")
parser.add_argument("--snapshot_period", type=float, default=1.0)
parser.add_argument("--maximum_snapshots", type=int, default=300)
args = parser.parse_args()


CSV_FIELDS = [
    "elapsed_s", "wall_time_s", "car_id", "frame_index", "detected",
    "confidence", "left_x_px", "right_x_px", "lane_center_x_px",
    "image_center_x_px", "center_offset_px", "center_offset_normalized",
    "heading_error_rad", "yellow_pixels", "white_pixels", "roi_top_px",
    "roi_bottom_px", "yellow_detected", "yellow_confidence",
    "yellow_near_x_px", "yellow_far_x_px", "yellow_offset_normalized",
    "yellow_heading_rad", "yellow_curvature", "yellow_row_coverage",
    "yellow_fit_rmse_px", "yellow_temporal_jump_px", "raw_snapshot_path",
    "yellow_near_y_px", "yellow_far_y_px", "yellow_extrapolated",
    "perspective_calibrated", "yellow_boundary_warning",
    "yellow_warning_enter_streak", "yellow_warning_exit_streak",
    "annotated_snapshot_path", "snapshot_path",
]


def value(number):
    try:
        return f"{float(number):.9f}"
    except (TypeError, ValueError):
        return "nan"


camera = QCarCameras(
    frameWidth=args.width,
    frameHeight=args.height,
    frameRate=args.frame_rate,
    enableFront=True,
)
detector = FrontLaneDetector()
boundary_warning = LaneBoundaryWarning()
probe = None
if args.probing:
    probe = Probe(ip=args.ip_host)
    probe.add_display(
        imageSize=[args.height, args.width, 3],
        scaling=True,
        scalingFactor=4,
        name="Front Lane Measurement",
    )

raw_directory = os.path.join(args.snapshot_directory, "raw")
annotated_directory = os.path.join(args.snapshot_directory, "annotated")
os.makedirs(raw_directory, exist_ok=True)
os.makedirs(annotated_directory, exist_ok=True)
csv_file = open(args.csv_path, "w", newline="", encoding="utf-8")
writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
writer.writeheader()
csv_file.flush()

start_time = time.time()
last_snapshot = float("-inf")
snapshot_count = 0
frame_index = 0

try:
    while True:
        flags = camera.readAll()
        if not any(flags):
            continue
        frame_index += 1
        elapsed = time.time() - start_time
        frame = camera.csiFront.imageData
        measurement, mask = detector.process(frame)
        warning_active = boundary_warning.update(
            measurement.yellow_detected,
            measurement.yellow_confidence,
            measurement.yellow_offset_normalized,
        )
        annotated = detector.render(frame, measurement, mask)
        if warning_active:
            cv2.putText(
                annotated,
                "YELLOW BOUNDARY PROXIMITY WARNING",
                (20, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        raw_snapshot_path = ""
        annotated_snapshot_path = ""
        if (
            snapshot_count < args.maximum_snapshots
            and elapsed - last_snapshot >= args.snapshot_period
        ):
            filename = f"frame_{frame_index:06d}_{elapsed:09.3f}s.jpg"
            raw_snapshot_path = os.path.join(raw_directory, filename)
            annotated_snapshot_path = os.path.join(
                annotated_directory, filename
            )
            raw_saved = cv2.imwrite(raw_snapshot_path, frame)
            annotated_saved = cv2.imwrite(annotated_snapshot_path, annotated)
            if raw_saved and annotated_saved:
                snapshot_count += 1
                last_snapshot = elapsed
            else:
                raw_snapshot_path = ""
                annotated_snapshot_path = ""

        writer.writerow({
            "elapsed_s": value(elapsed),
            "wall_time_s": value(time.time()),
            "car_id": args.car_id,
            "frame_index": frame_index,
            "detected": int(measurement.detected),
            "confidence": value(measurement.confidence),
            "left_x_px": value(measurement.left_x_px),
            "right_x_px": value(measurement.right_x_px),
            "lane_center_x_px": value(measurement.lane_center_x_px),
            "image_center_x_px": value(measurement.image_center_x_px),
            "center_offset_px": value(measurement.center_offset_px),
            "center_offset_normalized": value(measurement.center_offset_normalized),
            "heading_error_rad": value(measurement.heading_error_rad),
            "yellow_pixels": measurement.yellow_pixels,
            "white_pixels": measurement.white_pixels,
            "roi_top_px": measurement.roi_top_px,
            "roi_bottom_px": measurement.roi_bottom_px,
            "yellow_detected": int(measurement.yellow_detected),
            "yellow_confidence": value(measurement.yellow_confidence),
            "yellow_near_x_px": value(measurement.yellow_near_x_px),
            "yellow_far_x_px": value(measurement.yellow_far_x_px),
            "yellow_offset_normalized": value(
                measurement.yellow_offset_normalized
            ),
            "yellow_heading_rad": value(measurement.yellow_heading_rad),
            "yellow_curvature": value(measurement.yellow_curvature),
            "yellow_row_coverage": value(measurement.yellow_row_coverage),
            "yellow_fit_rmse_px": value(measurement.yellow_fit_rmse_px),
            "yellow_temporal_jump_px": value(
                measurement.yellow_temporal_jump_px
            ),
            "yellow_near_y_px": value(measurement.yellow_near_y_px),
            "yellow_far_y_px": value(measurement.yellow_far_y_px),
            "yellow_extrapolated": int(measurement.yellow_extrapolated),
            "perspective_calibrated": int(
                measurement.perspective_calibrated
            ),
            "yellow_boundary_warning": int(warning_active),
            "yellow_warning_enter_streak": boundary_warning.enter_streak,
            "yellow_warning_exit_streak": boundary_warning.exit_streak,
            "raw_snapshot_path": raw_snapshot_path,
            "annotated_snapshot_path": annotated_snapshot_path,
            # Backward-compatible alias for older collection instructions.
            "snapshot_path": annotated_snapshot_path,
        })
        if frame_index % 10 == 0:
            csv_file.flush()

        if probe is not None:
            if not probe.connected:
                probe.check_connection()
            if probe.connected and frame_index % 2 == 0:
                probe.send(name="Front Lane Measurement", imageData=annotated)

        if frame_index % max(1, int(args.frame_rate)) == 0:
            print(
                "[LANE] "
                f"yellowDetected={measurement.yellow_detected}, "
                f"yellowConfidence={measurement.yellow_confidence:.2f}, "
                f"yellowOffset={measurement.yellow_offset_normalized:.3f}, "
                f"yellowHeading={measurement.yellow_heading_rad:.3f} rad, "
                f"yellowWarning={warning_active}, "
                f"legacyTwoBoundary={measurement.detected}, "
                f"yellow={measurement.yellow_pixels}, "
                f"white={measurement.white_pixels}",
                flush=True,
            )
except KeyboardInterrupt:
    print("Front lane diagnostics interrupted.", flush=True)
finally:
    csv_file.close()
    camera.terminate()
    if probe is not None:
        probe.terminate()
