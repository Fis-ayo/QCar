# Codebase Cleanup Audit

Date: 2026-09-28

Status: Passes 1 and 2 completed; Pass 3 implemented and awaiting physical regression validation.

## Purpose

This audit is Pass 1 of a three-pass cleanup. It identifies what is required to
operate the two-QCar system, what supports repeatable research, and what belongs
to rejected experiments or generated output. No runtime file has been moved or
deleted as part of this audit.

The cleanup must preserve these working capabilities:

- start and stop both QCars from the local Windows computer;
- deploy the required software to each QCar;
- follow the configured fixed routes;
- coordinate access to CZ1;
- run YOLO obstacle detection;
- collect localization, tracking, and front-CSI lane measurements;
- reproduce and evaluate experiments for the supervisor report and paper.

## Active execution paths

### Start and deployment

`run_all_cars.bat` -> `python/start.py`

`python/start.py` currently:

1. starts `python/intersection_coordinator.py` on the local computer;
2. uploads every `qcar/*.py` and `qcar/*.json` file to each QCar;
3. optionally starts `python/observer.py` locally for the probing vehicle;
4. starts `qcar/yolo_server.py` remotely;
5. starts `qcar/lane_camera_server.py` remotely when configured;
6. starts `qcar/vehicle_control.py` remotely.

Pass 2 replaced the broad wildcard with an explicit runtime deployment
manifest. This prevents future analysis or archived experiment modules from
silently becoming part of the deployed system.

### Stop and calibration

- `stop_all_cars.bat` -> `python/stop.py`
- `reference_scan/calibrate.bat` -> `python/calibrate.py`

### Vehicle-control dependencies

`qcar/vehicle_control.py` directly uses:

- `utils.py`
- `road_rules.py` and `road_network.json`
- `intersection_client.py`
- `localization_diagnostics.py`
- `path_manager.py`
- `tracking_diagnostics.py`
- `lane_safety.py`

The rejected `curvature_control.py` and `footprint_correction.py` experiments
were removed from the active runtime in Pass 3. Their implementations remain
recoverable from Git tag `pre-runtime-cleanup`, and their results remain in
`EXPERIMENTAL_REPORTS.md`.

### Front-camera dependencies

`qcar/lane_camera_server.py` uses:

- `lane_detection.py` for measurement-only front-CSI lane detection;
- `lane_warning.py` for the diagnostic warning state;
- `lane_safety.py` for the current speed-only safety message.

The CSI result is not yet a calibrated lane-centering controller. It must not
be described as one in reports or code documentation.

## Classification

### Keep: operational entry points and configuration

| Files | Reason |
| --- | --- |
| `run_all_cars.bat`, `stop_all_cars.bat`, `ssh_setup.bat` | Local operator entry points. |
| `config.txt` | QCar and local-network configuration. |
| `python/start.py`, `python/stop.py` | Deployment and process lifecycle. |
| `python/intersection_coordinator.py` | Active CZ1 reservation service. |
| `python/observer.py` | Local probing display. |
| `python/calibrate.py`, `reference_scan/calibrate.bat` | Existing localization calibration workflow. |
| `qcar/vehicle_control.py` | Main navigation and control process. |
| `qcar/yolo_server.py`, `qcar/utils.py` | Obstacle detection and receiver/drive logic. |
| `qcar/road_rules.py`, `qcar/road_network.json` | Validated routes, zones, approaches, and stop-line definitions. |
| `qcar/path_manager.py` | Route-waypoint validation and heading repair. |
| `qcar/intersection_client.py` | Vehicle-side coordinator protocol and state machine. |
| `qcar/localization_diagnostics.py`, `qcar/tracking_diagnostics.py` | Required evidence collection and fault analysis. |
| `qcar/lane_camera_server.py`, `qcar/lane_detection.py` | Front-CSI measurement pipeline. |
| `qcar/lane_warning.py`, `qcar/lane_safety.py` | Current diagnostic warning and speed-only safety experiment. Keep until Pass 3 decides their final role. |

### Keep: documentation and research records

| Files | Reason |
| --- | --- |
| `README.md` | Installation and operating instructions. |
| `PROJECT_STATUS.md` | Current project state and next priorities. |
| `EXPERIMENTAL_REPORTS.md` | Chronological experimental evidence. |
| `docs/NEXT_SUPERVISOR_REPORT.txt` | Supervisor-facing progress report. |
| `docs/SOURCE_CODE_GUIDE.md` | Source-code appendix guide. |
| `docs/SYSTEM_ARCHITECTURE.md`, `docs/system_architecture.mmd`, `docs/system_architecture.png` | Architecture source and rendered figure. |
| `docs/LANE_TRACKING_BASELINE_PROTOCOL.md` | Repeatable baseline protocol. |
| `docs/LANE_KEEPING_ACCEPTANCE_PLAN.md` | Physical clearance criteria and evaluation gates. |
| `experiments/templates/trial_observations.csv` | Standard manual trial record. |

### Pass 2: organized maintained tools without changing algorithms

| Previous location | Current location | Purpose |
| --- | --- | --- |
| `python/analyze_*.py` | `tools/analysis/` | Offline log and trajectory analysis. |
| `python/evaluate_camera_clearance_signal.py` | `tools/analysis/` | Camera/physical-clearance evaluation. |
| `python/plot_*.py` | `tools/visualization/` | Map and result figures. |
| `python/render_system_architecture.py` | `tools/visualization/` | Architecture rendering. |
| `python/replay_*.py` | `tools/replay/` | Offline warning/safety replay. |
| `python/test_*.py` | `tests/` | Automated tests. |
| `phase0_cz1_candidate.png`, `validated_routes.png` | `docs/images/` | Documentation figures rather than runtime files. |

Imports and path discovery in moved scripts must be updated, then the complete
test suite must pass. Operational Python files remain under `python/`.

### Pass 2: generated or machine-local files

These remain ignored and should not be committed:

- `Logs/` and runtime CSV/log/frame output (preserved locally; do not remove);
- `intersection_coordinator.log`;
- `__pycache__/` and `*.pyc`;
- `.qcar_credentials.bat` and all credentials;
- machine-specific reference-scan `.mat` files unless a deliberate, documented
  decision is made to publish a non-sensitive calibration dataset.

The `Logs/` directory is experimental evidence and is explicitly outside the
cleanup removal scope. Temporary bytecode and regenerated process logs may be
cleared separately when appropriate, but Pass 2 did not remove them.

### Pass 3: runtime simplification implemented

1. Removed `curvature_control.py` and `footprint_correction.py`, their launcher
   flags, inactive controller branches, controller-specific diagnostic fields,
   and dedicated tests as one coherent change.
2. Removed the unused randomized-route variables/import. Valid routes continue
   to come from `road_network.json`.
3. Removed the unreachable automatic route-recovery branch. Calibration still
   exits after localization initialization; normal driving requires the
   launcher's explicit start node and retains the fail-closed pose check.
4. Kept `lane_warning.py` and `lane_safety.py` separate. The first preserves a
   measurement-only experimental event marker; the second owns speed
   restriction and UDP transport. Combining them would change responsibilities
   without improving the current physical evidence.
5. Preserved the baseline Stanley steering equation, fixed routes,
   intersection behavior, YOLO response, and lane-safety arbitration.

## Proposed explicit QCar deployment set

Pass 2 changed `python/start.py` to upload only:

```text
vehicle_control.py
yolo_server.py
utils.py
road_rules.py
road_network.json
path_manager.py
intersection_client.py
localization_diagnostics.py
tracking_diagnostics.py
lane_camera_server.py
lane_detection.py
lane_warning.py
lane_safety.py
```

## Three-pass acceptance checks

### Pass 1 — audit

- Every entry point and deployed dependency is identified.
- Every file has a keep, organize, generated, or review disposition.
- No runtime behavior changes.

### Pass 2 — low-risk organization

- Explicit deployment manifest replaces wildcard deployment.
- Tests and tools are moved with their imports corrected.
- Documentation links and commands are updated.
- Automated tests pass before and after the move.
- A launcher dry run or mocked deployment confirms the exact upload list.

Completed verification:

- 39 automated tests passed after the directory moves;
- all maintained Python files passed bytecode compilation;
- all entries in the explicit QCar deployment manifest exist;
- no QCar runtime file is omitted by the manifest; and
- `Logs/` was not moved, changed, or removed.

### Pass 3 — runtime cleanup

- Rejected/disabled controller branches are removed as one coherent change.
- Unit tests pass.
- Both QCars start, acquire the correct route, and produce expected logs.
- A controlled single-car baseline run confirms no behavioral regression.
- A controlled two-car CZ1 run confirms reservation behavior is unchanged.

Completed software verification:

- the remaining 31 automated tests pass;
- every maintained Python file passes bytecode compilation;
- the final 13-file QCar deployment manifest has no missing or omitted runtime
  dependencies;
- source diffs confirm that the baseline Stanley steering equation and nominal
  speed command are preserved; and
- `git diff --check` reports no whitespace errors.

The single-car and two-car physical regression checks remain pending.

## Pass 3 preservation and validation

The pre-cleanup implementation is preserved in commit `bac3583` and annotated
tag `pre-runtime-cleanup`. Historical raw logs remain local and ignored; they
were not reorganized or removed. Automated verification and controlled
physical regression checks are required before the cleanup is treated as the
new validated physical baseline.
