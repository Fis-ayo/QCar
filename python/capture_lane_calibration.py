"""Upload and run one motion-free front-CSI calibration capture on a QCar."""

import argparse
import os
import posixpath
import re
import shlex
import sys

import paramiko
from scp import SCPClient


def safe_label(value):
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    if not label or label in (".", ".."):
        raise argparse.ArgumentTypeError("label must contain a letter or number")
    return label


parser = argparse.ArgumentParser(
    description="Collect a labeled stationary yellow-divider camera sample."
)
parser.add_argument("--qcar_ip", default="192.168.0.151")
parser.add_argument("--remote_path", required=True)
parser.add_argument("--session", required=True, type=safe_label)
parser.add_argument("--pose", required=True, type=safe_label)
parser.add_argument("--road_section", required=True, type=safe_label)
parser.add_argument("--yellow_boundary_side", required=True,
                    choices=("left", "right"))
measurement_group = parser.add_mutually_exclusive_group(required=True)
measurement_group.add_argument(
    "--vehicle_center_to_yellow_mm", type=float,
    help="Measured QCar longitudinal-centerline to yellow-edge distance.",
)
measurement_group.add_argument(
    "--yellow_clearance_mm", type=float, dest="legacy_yellow_clearance_mm",
    help=argparse.SUPPRESS,
)
parser.add_argument("--duration_s", default=10.0, type=float)
args = parser.parse_args()

center_to_yellow_mm = args.vehicle_center_to_yellow_mm
if center_to_yellow_mm is None:
    center_to_yellow_mm = args.legacy_yellow_clearance_mm
    print(
        "WARNING: --yellow_clearance_mm is deprecated; interpreting it as "
        "vehicle-centerline-to-yellow distance.",
        file=sys.stderr,
    )
if center_to_yellow_mm < 0.0:
    parser.error("--vehicle_center_to_yellow_mm must be non-negative")
if not 3.0 <= args.duration_s <= 60.0:
    parser.error("--duration_s must be between 3 and 60 seconds")

password = os.environ.get("QCAR_PASSWORD")
if not password:
    raise RuntimeError("QCAR_PASSWORD is not set in the local environment")

current_dir = os.path.dirname(os.path.realpath(__file__))
qcar_dir = os.path.normpath(os.path.join(current_dir, "..", "qcar"))
runtime_files = (
    "lane_camera_server.py",
    "lane_detection.py",
    "lane_guidance.py",
    "lane_safety.py",
    "lane_warning.py",
)

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(args.qcar_ip, username="nvidia", password=password)

try:
    # Do not collect a supposedly stationary calibration while either the
    # vehicle controller or another camera process owns the hardware.
    check = (
        "if pgrep -f '[v]ehicle_control.py' >/dev/null; then echo VEHICLE_ACTIVE; "
        "elif pgrep -f '[l]ane_camera_server.py' >/dev/null; then echo CAMERA_ACTIVE; "
        "else echo SAFE; fi"
    )
    _, stdout, _ = ssh.exec_command(check)
    state = stdout.read().decode("utf-8", errors="replace").strip()
    if state != "SAFE":
        raise RuntimeError(
            "Calibration refused because remote process state is " + state
            + ". Stop the QCar applications first."
        )

    with SCPClient(ssh.get_transport()) as scp:
        for name in runtime_files:
            source = os.path.join(qcar_dir, name)
            if not os.path.isfile(source):
                raise FileNotFoundError(source)
            scp.put(source, args.remote_path)

    capture_name = f"{args.session}_{args.pose}"
    csv_name = f"yellow_calibration_{capture_name}.csv"
    frame_directory = f"yellow_calibration_{capture_name}_frames"
    remote_csv = posixpath.join(args.remote_path, csv_name)
    remote_frames = posixpath.join(args.remote_path, frame_directory)
    command = " ".join([
        "cd", shlex.quote(args.remote_path), "&&", "python",
        "lane_camera_server.py",
        "--car_id", shlex.quote(args.qcar_ip),
        "--calibration_mode",
        "--calibration_session", shlex.quote(args.session),
        "--calibration_pose", shlex.quote(args.pose),
        "--road_section", shlex.quote(args.road_section),
        "--yellow_boundary_side", shlex.quote(args.yellow_boundary_side),
        "--vehicle_center_to_yellow_mm", str(center_to_yellow_mm),
        "--duration_s", str(args.duration_s),
        "--csv_path", shlex.quote(csv_name),
        "--snapshot_directory", shlex.quote(frame_directory),
        "--snapshot_period", "1",
    ])
    print(
        f"Starting stationary capture {capture_name} on {args.qcar_ip}.\n"
        "The QCar controller is not running; no motion commands are sent.",
        flush=True,
    )
    _, stdout, stderr = ssh.exec_command(command)
    for line in iter(stdout.readline, ""):
        print(line.rstrip())
    exit_code = stdout.channel.recv_exit_status()
    error_text = stderr.read().decode("utf-8", errors="replace").strip()
    if exit_code != 0:
        raise RuntimeError(
            f"Remote capture failed with status {exit_code}: {error_text}"
        )
    print("Capture complete. Remote outputs:")
    print(f"  {remote_csv}")
    print(f"  {remote_frames}/")
finally:
    ssh.close()
