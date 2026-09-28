# Multi-QCar Autonomous Driving

This application runs two Quanser QCar2 vehicles on the SDCS physical map. It
combines fixed-route planning, EKF/GPS/LiDAR localization, Stanley-style path
tracking, YOLO-based obstacle response, and a centralized UDP intersection
reservation service.

The project is experimental. Every physical run requires an operator with an
accessible emergency stop.

## Current capabilities

- deterministic per-vehicle routes;
- validation and conservative repair of generated reference paths;
- one stop-controlled conflict zone (`CZ1_CENTRAL`);
- first-arrival reservation, stop, grant, occupancy, exit, and release states;
- fail-closed communication-loss handling and stale-owner recovery;
- YOLO-based obstacle/vehicle stopping;
- localization, path-tracking, intersection, and front-camera diagnostics; and
- front CSI yellow-line detection and an opt-in, speed-only lane-safety layer.
  Camera measurements never command steering; Candidate 5 can reduce speed or
  latch a stop on `.151` and is awaiting physical validation.

## Physical configuration

| Device | Address | Start node | Route |
|---|---|---:|---|
| Local Windows computer | `192.168.0.153` | — | Launcher, observer, and coordinator |
| QCar `.151` | `192.168.0.151` | 0 | `0 -> 2 -> 4 -> 6 -> 0` |
| QCar `.152` | `192.168.0.152` | 11 | `11 -> 12 -> 7 -> 14 -> 16 -> 18 -> 11` |

The assignments used at runtime are defined in `python/start.py`. Network and
deployment settings are read from `config.txt`.

## Prerequisites

- Windows computer with the Quanser QCar2/SDCS software environment installed;
- both QCars and the local computer connected to the same network;
- Python available from the Quanser environment;
- Python packages `paramiko` and `scp` on the local computer;
- QCar-side Quanser `pal`, `hal`, and `pit` packages; and
- SSH access to each configured QCar.

The launcher uses the standard laboratory QCar username configured in the
Python scripts. The password is read from the local `QCAR_PASSWORD` environment
variable and is never stored in the repository.

## Configure a run

Edit `config.txt` before launching:

```text
REMOTE_PATH=/home/nvidia/Documents/Quanser/applications/multi_vehicle_self_driving
QCAR_IPS=[192.168.0.151,192.168.0.152]
CALIBRATE_IP=192.168.0.152
PROBING_IP=192.168.0.151
LOCAL_IP=192.168.0.153
WIDTH=320
HEIGHT=200
```

`QCAR_IPS` controls which cars are launched. At the time this document was
written, the checked-in value contains only `.151`; retain that value for a
single-car baseline or add `.152` deliberately for a supervised two-car run.

`PROBING_IP` selects the vehicle whose live observer streams are displayed.
`CALIBRATE_IP` is used by the separate calibration workflow and does not cause
calibration during an ordinary run.

## First-time SSH preparation

From this directory, run:

```powershell
./ssh_setup.bat
```

This adds the configured QCar SSH host keys to the local known-hosts file.

## Start the application

1. Set the QCar SSH password in the current PowerShell session:

```powershell
$env:QCAR_PASSWORD = "your-laboratory-qcar-password"
```

2. Place each QCar at its configured start node and correct orientation.
3. Confirm that the intended IP addresses are present in `config.txt`.
4. Keep an emergency-stop method immediately available.
5. Run:

```powershell
./run_all_cars.bat
```

The launcher starts the local intersection coordinator, uploads the QCar-side
Python/JSON files, starts YOLO, optionally starts measurement-only CSI lane
diagnostics, and then starts vehicle control. A vehicle remains stopped if its
initial pose does not agree with its configured start node.

The current `.151` launcher configuration explicitly enables experimental
Candidate 5 lane safety. A reliable yellow boundary approaching the calibrated
safety margin reduces target speed; a critical threshold latches a complete stop until the
process is restarted. Loss of the local camera-safety data stream also stops
the car. Keep the emergency stop available and do not expect the vehicle to
resume automatically after a critical lane stop.

## Stop the application

```powershell
./stop_all_cars.bat
```

Confirm physically that both vehicles have stopped. The script terminates
remote Python processes and the locally managed coordinator process.

## Logs and experimental evidence

| Artifact | Location | Purpose |
|---|---|---|
| `intersection_coordinator.log` | Local project directory | Queue, grant, release, timeout, and recovery events |
| `vehicle_control.log` | Remote application directory on each QCar | Vehicle state, control, tracking, and intersection status |
| `yolo_server.log` | Remote QCar directory | YOLO startup/runtime diagnostics |
| `lane_camera.log` | Remote QCar directory when enabled | Front CSI detector status |
| `tracking_diagnostics.csv` | Remote QCar directory | Reference-path and control measurements |
| `localization_diagnostics.csv` | Remote QCar directory | Raw and EKF localization measurements |
| `lane_diagnostics.csv` | Remote QCar directory when enabled | Image-space lane measurements |
| `lane_frames/` | Remote QCar directory when enabled | Raw and annotated diagnostic frames |
| `localization_events/` | Remote QCar directory | Captured large-innovation LiDAR events, if any |

Archive configuration, code revision, video, and physical observations with
each formal experiment. See `EXPERIMENTAL_REPORTS.md` for the experiment record.

## Local verification

Run the complete local test suite:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

Validate the road configuration separately with:

```powershell
python qcar/road_rules.py
```

Maintained offline utilities are grouped under `tools/analysis/`,
`tools/visualization/`, and `tools/replay/`. Automated tests are under
`tests/`. Experimental run evidence remains under the local, Git-ignored
`Logs/` directory and is not removed by source-code cleanup.

## Documentation

- `docs/SYSTEM_ARCHITECTURE.md` — component/data-flow diagrams and safety ownership;
- `docs/SOURCE_CODE_GUIDE.md` — implementation inventory for the report appendix;
- `docs/LANE_TRACKING_BASELINE_PROTOCOL.md` — controlled baseline procedure, metrics, and acceptance gates;
- `PROJECT_STATUS.md` — current roadmap, status, limitations, and decisions; and
- `EXPERIMENTAL_REPORTS.md` — chronological experiments and results.
