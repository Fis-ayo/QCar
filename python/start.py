import paramiko
import argparse
from scp import SCPClient
import subprocess
import os
import time
import atexit
import sys

parser = argparse.ArgumentParser(description="Run a demo script on a remote server.")
parser.add_argument('-ip', '--qcar_ips', type=str, required=True, help="IP addresses of QCars")
parser.add_argument('-rp','--remote_path',type=str,required=True, help="Remote path on QCar")
parser.add_argument('-ipp','--probing_ip', type=str, required=True, help="IP addresses of QCar to probe")
parser.add_argument('-ipl','--local_ip', type=str, required=True,help="IP addresses of local machine")
parser.add_argument('-w','--width', default=320, help="width of to image to be displayed in the observer")
parser.add_argument('-ht','--height', default=200, help="height of to image to be displayed in the observer")
args = parser.parse_args()

QCAR_IPS = args.qcar_ips.split(",")  # Comma-separated list of IPs
LOCAL_IP = args.local_ip
PROBING_IP = args.probing_ip
WIDTH = args.width
HEIGHT = args.height
REMOTE_PATH = args.remote_path
USERNAME = "nvidia"
PASSWORD = os.environ.get("QCAR_PASSWORD")
LOCAL_SCRIPTS_PATH = "../qcar"
LOCAL_OBSERVER_PATH = "python/observer.py"
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
SCRIPTS_PATH = os.path.normpath(os.path.join(CURRENT_DIR,LOCAL_SCRIPTS_PATH))
QCAR_RUNTIME_FILES = (
    "vehicle_control.py",
    "yolo_server.py",
    "utils.py",
    "road_rules.py",
    "road_network.json",
    "path_manager.py",
    "intersection_client.py",
    "localization_diagnostics.py",
    "tracking_diagnostics.py",
    "lane_camera_server.py",
    "lane_detection.py",
    "lane_warning.py",
    "lane_safety.py",
    # Transitional dependencies. Pass 3 will remove these after their disabled
    # controller branches are removed from vehicle_control.py.
    "curvature_control.py",
    "footprint_correction.py",
)
COORDINATOR_PORT = 18750
COORDINATOR_SCRIPT = os.path.join(CURRENT_DIR, "intersection_coordinator.py")
COORDINATOR_LOG_PATH = os.path.normpath(
    os.path.join(CURRENT_DIR, "..", "intersection_coordinator.log")
)
COORDINATOR_PID_PATH = os.path.normpath(
    os.path.join(CURRENT_DIR, "..", ".intersection_coordinator.pid")
)

# Explicit assignments prevent a change in QCAR_IPS ordering from silently
# changing a vehicle's route. These start nodes match the physical setup.
CAR_SETTINGS = {
    "192.168.0.152": {
        "node_configuration": 1,
        "start_node": 11,
        # Keep .152 on baseline until candidate 2 passes on .151.
        "curvature_control": False,
        "footprint_correction": False,
        "footprint_correction_gain": 1.0,
        "footprint_correction_max_m": 0.04,
        # Preserve the known baseline for the other vehicle.
        "tracking_offset_m": 0.20,
        # Introduce camera measurement on one car before fleet-wide use.
        "lane_camera": False,
        "lane_safety": False,
    },
    "192.168.0.151": {
        "node_configuration": 0,
        "start_node": 0,
        # Candidate 3 improved numerical error but still crossed physically.
        "curvature_control": False,
        # Candidate 1 (gain 1.0, max 0.04 m) delayed but did not prevent the
        # crossing. Candidate 2 (gain 1.65, max 0.06 m) still crossed and made
        # the outer-edge behavior noticeably worse. Preserve the settings for
        # reproducibility, but keep the rejected correction disabled.
        # "footprint_correction": True,
        "footprint_correction": False,
        "footprint_correction_gain": 1.65,
        "footprint_correction_max_m": 0.06,
        # Controlled EXP-019 test: documented body-center-to-front-axle
        # distance. Do not change gains, speed, or route during this run.
        "tracking_offset_m": 0.13,
        # Measurement only: this process cannot command steering or throttle.
        "lane_camera": True,
        # Candidate 5: earlier speed-only CSI slowdown and latched safety stop.
        # This cannot modify steering and remains disabled by default in
        # vehicle_control.py unless this explicit launcher flag is supplied.
        "lane_safety": True,
    },
}

# --- Helper to create SSH + SCP connections ---
def create_ssh_and_scp(ip):
    if not PASSWORD:
        raise RuntimeError(
            "QCAR_PASSWORD is not set. Set it in the local environment "
            "before launching the QCars."
        )
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=USERNAME, password=PASSWORD)
    scp = SCPClient(ssh.get_transport())
    return ssh, scp


coordinator_log = open(COORDINATOR_LOG_PATH, "w", encoding="utf-8")
coordinator_process = subprocess.Popen(
    [
        sys.executable,
        "-u",
        COORDINATOR_SCRIPT,
        "--port",
        str(COORDINATOR_PORT),
    ],
    stdout=coordinator_log,
    stderr=subprocess.STDOUT,
)
with open(COORDINATOR_PID_PATH, "w", encoding="utf-8") as pid_file:
    pid_file.write(str(coordinator_process.pid))


def stop_coordinator():
    if coordinator_process.poll() is None:
        coordinator_process.terminate()
    coordinator_log.close()
    try:
        os.remove(COORDINATOR_PID_PATH)
    except FileNotFoundError:
        pass


atexit.register(stop_coordinator)
time.sleep(1)
if coordinator_process.poll() is not None:
    raise RuntimeError(
        f"Intersection coordinator failed to start. See {COORDINATOR_LOG_PATH}"
    )
print(
    f"Intersection coordinator started on UDP port {COORDINATOR_PORT}. "
    f"Log: {COORDINATOR_LOG_PATH}"
)

for ip in QCAR_IPS:
    print(f"\n Starting QCar: {ip}")

    if ip not in CAR_SETTINGS:
        raise ValueError(
            f"No route/start-node settings configured for QCar {ip}. "
            "Add it to CAR_SETTINGS in python/start.py."
        )

    # Upload Python scripts
    ssh, scp = create_ssh_and_scp(ip)

    deploy_files = [
        os.path.normpath(os.path.join(SCRIPTS_PATH, name))
        for name in QCAR_RUNTIME_FILES
    ]
    missing_files = [file for file in deploy_files if not os.path.isfile(file)]
    if missing_files:
        raise FileNotFoundError(
            "QCar deployment manifest contains missing files: "
            + ", ".join(missing_files)
        )
    for file in deploy_files:
        scp.put(file, REMOTE_PATH)
    print(f"[{ip}] Uploaded QCar scripts and road configuration.")

    # Set PROBING flag
    is_probing = (ip == PROBING_IP)
    probing_flag = "True" if is_probing else "False"

    # Start observer.py locally if this is the probing QCar
    if is_probing:
        print(f"[{ip}] Starting observer.py locally (probing mode).")
        subprocess.Popen(["python", LOCAL_OBSERVER_PATH])

    # Start the YOLO publisher first. Keep it alive after SSH disconnects and
    # retain its output so remote startup failures can be diagnosed.
    cmd_yolo = (
        f"cd {REMOTE_PATH} && "
        f"nohup python yolo_server.py -p {probing_flag} -i {LOCAL_IP} "
        f"-w {WIDTH} -ht {HEIGHT} > yolo_server.log 2>&1 < /dev/null &"
    )
    ssh.exec_command(cmd_yolo)
    print(f"[{ip}] Started yolo_server.py.")

    settings = CAR_SETTINGS[ip]
    if settings.get("lane_camera", False):
        lane_probe_flag = "--probing" if is_probing else ""
        lane_safety_publish_flag = (
            "--lane_safety_publish"
            if settings.get("lane_safety", False) else ""
        )
        cmd_lane = (
            f"cd {REMOTE_PATH} && "
            f"nohup python lane_camera_server.py "
            f"--car_id {ip} --ip_host {LOCAL_IP} {lane_probe_flag} "
            f"{lane_safety_publish_flag} "
            f"> lane_camera.log 2>&1 < /dev/null &"
        )
        ssh.exec_command(cmd_lane)
        print(
            f"[{ip}] Started front CSI lane diagnostics "
            f"(speed safety={settings.get('lane_safety', False)})."
        )

    time.sleep(3)

    # Use the route and physical start node assigned to this specific QCar.
    # nohup prevents the controller from being killed when SSH disconnects.
    node_configuration = settings["node_configuration"]
    start_node = settings["start_node"]
    curvature_flag = (
        "--curvature_control" if settings.get("curvature_control", False) else ""
    )
    footprint_flag = (
        "--footprint_correction"
        if settings.get("footprint_correction", False)
        else ""
    )
    footprint_gain = settings.get("footprint_correction_gain", 1.0)
    footprint_max_m = settings.get("footprint_correction_max_m", 0.04)
    tracking_offset_m = settings.get("tracking_offset_m", 0.20)
    lane_safety_flag = (
        "--lane_safety" if settings.get("lane_safety", False) else ""
    )
    cmd_vehicle = (
        f"cd {REMOTE_PATH} && "
        f"nohup python vehicle_control.py -n {node_configuration} "
        f"-s {start_node} "
        f"--coordinator_ip {LOCAL_IP} "
        f"--coordinator_port {COORDINATOR_PORT} "
        f"--car_id {ip} "
        f"--tracking_offset_m {tracking_offset_m} "
        f"--footprint_correction_gain {footprint_gain} "
        f"--footprint_correction_max_m {footprint_max_m} "
        f"{curvature_flag} {footprint_flag} {lane_safety_flag} "
        f"> vehicle_control.log 2>&1 < /dev/null &"
    )
    ssh.exec_command(cmd_vehicle)
    print(
        f"[{ip}] Started vehicle_control.py "
        f"(node configuration {node_configuration}, "
        f"start node {start_node}, "
        f"tracking offset {tracking_offset_m:.3f} m)."
    )

    ssh.close()
    scp.close()

    time.sleep(1)

input("\n All QCars started, press Enter to exit...")  # Wait for user input before exiting
