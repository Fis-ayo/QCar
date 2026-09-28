# Source Code Guide

This inventory identifies the implementation files requested for technical
documentation and a future report appendix. Generated logs, recorded frames,
cache files, and proprietary Quanser libraries should not be copied into the
source appendix.

## Launch, deployment, and shutdown

| File | Responsibility |
|---|---|
| `run_all_cars.bat` | Reads `config.txt` and invokes the local launcher |
| `python/start.py` | Starts the coordinator, deploys QCar files with SCP, and launches per-car processes through SSH |
| `stop_all_cars.bat` | Invokes remote/local shutdown and stops QUARC models |
| `python/stop.py` | Terminates QCar Python processes and the managed local coordinator |
| `ssh_setup.bat` | Registers configured QCar SSH host keys |
| `config.txt` | Defines QCar addresses, local address, remote directory, and probing selection |

## Road model, routes, and reference trajectories

| File | Responsibility |
|---|---|
| `qcar/road_network.json` | Machine-readable routes, available roads, CZ1 geometry, stop/exit regions, and traffic rules |
| `qcar/road_rules.py` | Loads and validates the road-network configuration |
| `qcar/path_manager.py` | Validates generated waypoint geometry and conservatively repairs supported paired reversals |
| `qcar/footprint_correction.py` | Computes the optional bounded outward curve-reference displacement used to test vehicle off-tracking |
| `tools/visualization/plot_validated_routes.py` | Compares raw and prepared routes offline |
| `tools/visualization/plot_phase0_zones.py` | Visualizes configured conflict-zone geometry |

## Vehicle localization and control

| File | Responsibility |
|---|---|
| `qcar/vehicle_control.py` | Creates the route, estimates state, advances waypoints, computes speed/steering, applies safety restrictions, and writes actuator commands |
| `qcar/localization_diagnostics.py` | Logs raw GPS/LiDAR and EKF behavior and captures significant localization events |
| `qcar/tracking_diagnostics.py` | Logs vehicle pose, projected route reference, tracking errors, commands, and intersection state |
| `qcar/curvature_control.py` | Contains experimental curvature-control helpers; not enabled in the current launcher configuration |
| `tools/analysis/analyze_tracking.py` | Produces offline summaries/plots from tracking diagnostics |

The current steering command is map/path based. The front CSI measurement
pipeline has no steering or throttle output.

## Intersection coordination and communication recovery

| File | Responsibility |
|---|---|
| `python/intersection_coordinator.py` | Maintains the CZ1 request queue, ownership, release, fairness, stale-owner lock, and recovery behavior |
| `qcar/intersection_client.py` | Sends non-blocking UDP vehicle state and implements `CRUISING`, `APPROACHING`, `WAITING`, `GRANTED`, `OCCUPYING`, and `EXITED` |

The coordinator is centralized on the local computer. QCars transmit their
zone state and receive the current owner/grant decision. On communication loss,
the logic fails closed before entry. A vehicle already occupying CZ1 continues
clearing rather than stopping inside the conflict zone.

## YOLO obstacle detection

| File | Responsibility |
|---|---|
| `qcar/yolo_server.py` | Reads aligned RGB/depth data, runs YOLOv8, and publishes processed detections |
| `qcar/utils.py` | Provides the YOLO publisher/receiver and vehicle-side drive restriction logic |
| `python/observer.py` | Displays the probing vehicle's YOLO and lane diagnostic streams |

YOLO provides an independent collision safeguard. An intersection grant does
not override an obstacle stop.

## Front CSI lane diagnostics

| File | Responsibility |
|---|---|
| `qcar/lane_camera_server.py` | Captures front CSI frames, logs measurements, and saves raw/annotated snapshots |
| `qcar/lane_detection.py` | Detects visible yellow lane features and reports bounded image-space measurements/confidence |
| `qcar/lane_warning.py` | Debounces and latches the measurement-only yellow-boundary proximity warning |
| `qcar/lane_safety.py` | Produces confidence-gated slowdown/latched-stop states and transports them locally over UDP |
| `tools/analysis/analyze_lane_frames.py` | Reprocesses saved frames offline without influencing vehicle control |
| `tools/analysis/analyze_camera_tracking_alignment.py` | Time-aligns CSI measurements with tracking diagnostics |
| `tools/analysis/evaluate_camera_clearance_signal.py` | Evaluates candidate camera warning features against diagnostic event proxies |
| `tools/replay/replay_lane_warning.py` | Replays the runtime warning logic on recorded runs |
| `tools/replay/replay_lane_safety.py` | Replays Candidate 4 slowdown/stop timing on recorded runs |

The detector remains non-steering. Candidate 5 may restrict speed or latch a
stop, but the camera cannot select a route or issue a steering command.

Lane-keeping objectives, physical-clearance definitions, staged acceptance
gates, and the next calibration experiment are defined in
`docs/LANE_KEEPING_ACCEPTANCE_PLAN.md`.

## Automated tests

| File | Coverage |
|---|---|
| `tests/test_phase1.py` | Reservation order, release, fairness, stale-owner recovery, and vehicle intersection states |
| `tests/test_path_manager.py` | Path validation, anchor preservation, supported repair, and fail-closed behavior |
| `tests/test_localization_diagnostics.py` | Localization CSV and event capture |
| `tests/test_tracking_diagnostics.py` | Tracking recorder calculations and output |
| `tests/test_lane_detection.py` | Image-space lane detector behavior |
| `tests/test_lane_warning.py` | Warning confidence gate, debounce, latch, and optional hysteresis |
| `tests/test_lane_safety.py` | Slowdown/stop debounce, stop latch, confidence rejection, release margin, and stale-transport stop |
| `tests/test_curvature_control.py` | Experimental curvature-control helpers |

Run all tests with:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

## Suggested report appendix organization

1. Configuration and launcher (`config.txt`, `run_all_cars.bat`, relevant parts
   of `python/start.py`).
2. Road and route definition (`road_network.json`, `road_rules.py`,
   `path_manager.py`).
3. Vehicle localization and control (`vehicle_control.py` and diagnostic
   modules).
4. Intersection coordination (`intersection_coordinator.py` and
   `intersection_client.py`).
5. YOLO safety (`yolo_server.py` and relevant `utils.py` classes).
6. Communication recovery implementation and tests.
7. Test procedure, configuration revision, and quantitative results.

For a paper or formal report, include concise relevant excerpts in the
appendix and link to the complete repository rather than reproducing every
support file in the main body.
