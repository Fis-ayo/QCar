import paramiko
import argparse
import os
import signal


# Configuration
parser = argparse.ArgumentParser(description="Stop all applications on QCars.")
parser.add_argument('-ip', '--qcar_ips', type=str, required=True, help="IP addresses of QCars")
args = parser.parse_args()
QCAR_IPS = args.qcar_ips.split(",")  # Comma-separated list of IPs
USERNAME = "nvidia"
PASSWORD = os.environ.get("QCAR_PASSWORD")
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
COORDINATOR_PID_PATH = os.path.normpath(
    os.path.join(CURRENT_DIR, "..", ".intersection_coordinator.pid")
)

def create_ssh(ip):
    if not PASSWORD:
        raise RuntimeError(
            "QCAR_PASSWORD is not set. Set it in the local environment "
            "before stopping the QCars."
        )
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ip, username=USERNAME, password=PASSWORD)
    return ssh

for ip in QCAR_IPS:
    ssh = create_ssh(ip)
    ssh.exec_command("pkill -2 python ; pkill -15 python")
    ssh.close()

# Stop the automatic coordinator running on this local machine.
if os.path.exists(COORDINATOR_PID_PATH):
    try:
        with open(COORDINATOR_PID_PATH, "r", encoding="utf-8") as pid_file:
            coordinator_pid = int(pid_file.read().strip())
        os.kill(coordinator_pid, signal.SIGTERM)
        print(f"Stopped local intersection coordinator ({coordinator_pid}).")
    except (OSError, ValueError) as error:
        print(f"Unable to stop local intersection coordinator: {error}")
    finally:
        try:
            os.remove(COORDINATOR_PID_PATH)
        except FileNotFoundError:
            pass
