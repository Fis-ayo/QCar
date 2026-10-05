"""Run measurement-only lane diagnostics using the QCar2 front CSI camera."""

import argparse
import csv
import os
import time

import cv2
from pal.products.qcar import QCarCameras
from pal.utilities.probe import Probe

from lane_detection import FrontLaneDetector
from lane_guidance import LANE_GUIDANCE_PORT, LaneGuidancePublisher
from lane_safety import LANE_SAFETY_PORT, LaneSafetyMonitor, LaneSafetyPublisher
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
parser.add_argument("--lane_safety_publish", action="store_true")
parser.add_argument("--lane_safety_port", type=int, default=LANE_SAFETY_PORT)
parser.add_argument("--lane_guidance_publish", action="store_true")
parser.add_argument("--lane_guidance_port", type=int, default=LANE_GUIDANCE_PORT)
parser.add_argument(
    "--calibration_mode", action="store_true",
    help="Stationary labeled capture; never publishes safety/guidance packets.",
)
parser.add_argument("--calibration_session", default="")
parser.add_argument("--calibration_pose", default="")
parser.add_argument("--road_section", default="")
parser.add_argument(
    "--yellow_boundary_side", choices=("left", "right", "unknown"),
    default="unknown",
)
measurement_group = parser.add_mutually_exclusive_group()
measurement_group.add_argument(
    "--vehicle_center_to_yellow_mm", type=float, default=None,
    help="Measured QCar longitudinal-centerline to yellow-edge distance.",
)
measurement_group.add_argument(
    "--yellow_clearance_mm", type=float,
    dest="legacy_yellow_clearance_mm", default=None,
    help=argparse.SUPPRESS,
)
parser.add_argument(
    "--duration_s", type=float, default=0.0,
    help="Stop automatically after this many seconds; zero runs continuously.",
)
args = parser.parse_args()

if args.calibration_mode:
    center_to_yellow_mm = args.vehicle_center_to_yellow_mm
    if center_to_yellow_mm is None:
        center_to_yellow_mm = args.legacy_yellow_clearance_mm
        if center_to_yellow_mm is not None:
            print(
                "[LANE] WARNING: --yellow_clearance_mm is deprecated; "
                "interpreting it as vehicle-centerline-to-yellow distance.",
                flush=True,
            )
    missing = []
    if not args.calibration_session.strip():
        missing.append("--calibration_session")
    if not args.calibration_pose.strip():
        missing.append("--calibration_pose")
    if not args.road_section.strip():
        missing.append("--road_section")
    if center_to_yellow_mm is None or center_to_yellow_mm < 0.0:
        missing.append("--vehicle_center_to_yellow_mm (non-negative)")
    if args.duration_s <= 0.0:
        missing.append("--duration_s (positive)")
    if missing:
        parser.error("calibration mode requires " + ", ".join(missing))
    # Calibration is observation-only even if publishing flags are supplied.
    args.lane_safety_publish = False
    args.lane_guidance_publish = False
else:
    center_to_yellow_mm = None


CSV_FIELDS = [
    "elapsed_s", "wall_time_s", "car_id", "frame_index", "detected",
    "capture_mode", "calibration_session", "calibration_pose",
    "road_section", "yellow_boundary_side", "vehicle_center_to_yellow_mm",
    "confidence", "left_x_px", "right_x_px", "lane_center_x_px",
    "image_center_x_px", "center_offset_px", "center_offset_normalized",
    "heading_error_rad", "yellow_pixels", "white_pixels", "roi_top_px",
    "roi_bottom_px", "yellow_detected", "yellow_confidence",
    "yellow_near_x_px", "yellow_far_x_px", "yellow_offset_normalized",
    "yellow_heading_rad", "yellow_curvature", "yellow_row_coverage",
    "yellow_fit_rmse_px", "yellow_temporal_jump_px", "raw_snapshot_path",
    "yellow_near_y_px", "yellow_far_y_px", "yellow_extrapolated",
    "yellow_reference_detected", "yellow_reference_x_px",
    "yellow_reference_y_px", "yellow_reference_confidence",
    "yellow_multirow_fractions", "yellow_multirow_detected",
    "yellow_multirow_x_px", "yellow_multirow_confidence",
    "corridor_detected", "corridor_confidence", "corridor_paired_rows",
    "corridor_yellow_near_x_px", "corridor_white_near_x_px",
    "corridor_center_near_x_px", "corridor_offset_normalized",
    "corridor_heading_rad",
    "corridor_outer_source",
    "quanser_yellow_detected", "quanser_yellow_confidence",
    "quanser_yellow_slope", "quanser_yellow_intercept",
    "quanser_yellow_fit_rmse_px", "quanser_shadow_steering_rad",
    "corridor_trend_state", "corridor_trend_offset_delta",
    "corridor_trend_heading_delta", "corridor_trend_history_count",
    "perspective_calibrated", "yellow_boundary_warning",
    "yellow_warning_enter_streak", "yellow_warning_exit_streak",
    "lane_safety_state", "lane_safety_speed_gain",
    "lane_safety_reliable", "lane_safety_stop_latched",
    "lane_safety_slowdown_streak", "lane_safety_stop_streak",
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
safety_monitor = LaneSafetyMonitor()
safety_publisher = (
    LaneSafetyPublisher(port=args.lane_safety_port, car_id=args.car_id)
    if args.lane_safety_publish else None
)
guidance_publisher = (
    LaneGuidancePublisher(port=args.lane_guidance_port, car_id=args.car_id)
    if args.lane_guidance_publish else None
)
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
        if args.duration_s > 0.0 and time.time() - start_time >= args.duration_s:
            print(
                f"[LANE] Capture complete after {args.duration_s:.1f} s.",
                flush=True,
            )
            break
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
        safety = safety_monitor.update(
            measurement.yellow_detected,
            measurement.yellow_confidence,
            measurement.yellow_offset_normalized,
        )
        if safety_publisher is not None:
            safety_publisher.send(safety)
        if guidance_publisher is not None:
            guidance_publisher.send({
                "detected": measurement.detected,
                "confidence": measurement.confidence,
                "center_offset_normalized": (
                    measurement.center_offset_normalized
                ),
                "heading_error_rad": measurement.heading_error_rad,
                "left_x_px": measurement.left_x_px,
                "right_x_px": measurement.right_x_px,
                "lane_center_x_px": measurement.lane_center_x_px,
                "perspective_calibrated": measurement.perspective_calibrated,
                "calibration_id": "none",
                "yellow_reference_detected": (
                    measurement.yellow_reference_detected
                ),
                "yellow_reference_x_px": measurement.yellow_reference_x_px,
                "yellow_reference_y_px": measurement.yellow_reference_y_px,
                "yellow_reference_confidence": (
                    measurement.yellow_reference_confidence
                ),
                "yellow_multirow_fractions": measurement.yellow_multirow_fractions,
                "yellow_multirow_detected": measurement.yellow_multirow_detected,
                "yellow_multirow_x_px": measurement.yellow_multirow_x_px,
                "yellow_multirow_confidence": measurement.yellow_multirow_confidence,
                "corridor_detected": measurement.corridor_detected,
                "corridor_confidence": measurement.corridor_confidence,
                "corridor_paired_rows": measurement.corridor_paired_rows,
                "corridor_offset_normalized": (
                    measurement.corridor_offset_normalized
                ),
                "corridor_heading_rad": measurement.corridor_heading_rad,
                "corridor_outer_source": measurement.corridor_outer_source,
                "quanser_yellow_detected": measurement.quanser_yellow_detected,
                "quanser_yellow_confidence": measurement.quanser_yellow_confidence,
                "quanser_yellow_slope": measurement.quanser_yellow_slope,
                "quanser_yellow_intercept": measurement.quanser_yellow_intercept,
                "quanser_shadow_steering_rad": (
                    measurement.quanser_shadow_steering_rad
                ),
                "corridor_trend_state": measurement.corridor_trend_state,
                "corridor_trend_offset_delta": (
                    measurement.corridor_trend_offset_delta
                ),
                "corridor_trend_heading_delta": (
                    measurement.corridor_trend_heading_delta
                ),
            })
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
        if safety["state"] == LaneSafetyMonitor.SLOWDOWN:
            cv2.putText(
                annotated, "LANE SAFETY: SLOWDOWN", (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2,
            )
        elif safety["state"] == LaneSafetyMonitor.STOP:
            cv2.putText(
                annotated, "LANE SAFETY: STOP LATCHED", (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
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
            "capture_mode": (
                "stationary_calibration" if args.calibration_mode
                else "driving_diagnostics"
            ),
            "calibration_session": args.calibration_session,
            "calibration_pose": args.calibration_pose,
            "road_section": args.road_section,
            "yellow_boundary_side": args.yellow_boundary_side,
            "vehicle_center_to_yellow_mm": value(center_to_yellow_mm),
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
            "yellow_reference_detected": int(
                measurement.yellow_reference_detected
            ),
            "yellow_reference_x_px": value(
                measurement.yellow_reference_x_px
            ),
            "yellow_reference_y_px": value(
                measurement.yellow_reference_y_px
            ),
            "yellow_reference_confidence": value(
                measurement.yellow_reference_confidence
            ),
            "yellow_multirow_fractions": ";".join(
                value(item) for item in measurement.yellow_multirow_fractions
            ),
            "yellow_multirow_detected": ";".join(
                str(int(item)) for item in measurement.yellow_multirow_detected
            ),
            "yellow_multirow_x_px": ";".join(
                value(item) for item in measurement.yellow_multirow_x_px
            ),
            "yellow_multirow_confidence": ";".join(
                value(item) for item in measurement.yellow_multirow_confidence
            ),
            "corridor_detected": int(measurement.corridor_detected),
            "corridor_confidence": value(measurement.corridor_confidence),
            "corridor_paired_rows": measurement.corridor_paired_rows,
            "corridor_yellow_near_x_px": value(
                measurement.corridor_yellow_near_x_px
            ),
            "corridor_white_near_x_px": value(
                measurement.corridor_white_near_x_px
            ),
            "corridor_center_near_x_px": value(
                measurement.corridor_center_near_x_px
            ),
            "corridor_offset_normalized": value(
                measurement.corridor_offset_normalized
            ),
            "corridor_heading_rad": value(measurement.corridor_heading_rad),
            "corridor_outer_source": measurement.corridor_outer_source,
            "quanser_yellow_detected": int(
                measurement.quanser_yellow_detected
            ),
            "quanser_yellow_confidence": value(
                measurement.quanser_yellow_confidence
            ),
            "quanser_yellow_slope": value(measurement.quanser_yellow_slope),
            "quanser_yellow_intercept": value(
                measurement.quanser_yellow_intercept
            ),
            "quanser_yellow_fit_rmse_px": value(
                measurement.quanser_yellow_fit_rmse_px
            ),
            "quanser_shadow_steering_rad": value(
                measurement.quanser_shadow_steering_rad
            ),
            "corridor_trend_state": measurement.corridor_trend_state,
            "corridor_trend_offset_delta": value(
                measurement.corridor_trend_offset_delta
            ),
            "corridor_trend_heading_delta": value(
                measurement.corridor_trend_heading_delta
            ),
            "corridor_trend_history_count": (
                measurement.corridor_trend_history_count
            ),
            "perspective_calibrated": int(
                measurement.perspective_calibrated
            ),
            "yellow_boundary_warning": int(warning_active),
            "yellow_warning_enter_streak": boundary_warning.enter_streak,
            "yellow_warning_exit_streak": boundary_warning.exit_streak,
            "lane_safety_state": safety["state"],
            "lane_safety_speed_gain": value(safety["speed_gain"]),
            "lane_safety_reliable": int(safety["reliable"]),
            "lane_safety_stop_latched": int(safety["stop_latched"]),
            "lane_safety_slowdown_streak": safety["slowdown_streak"],
            "lane_safety_stop_streak": safety["stop_streak"],
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
                f"laneSafety={safety['state']}, "
                f"laneGain={safety['speed_gain']:.2f}, "
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
    if safety_publisher is not None:
        safety_publisher.close()
    if guidance_publisher is not None:
        guidance_publisher.close()
