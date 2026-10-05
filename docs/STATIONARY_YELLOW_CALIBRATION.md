# Stationary Yellow-Divider Calibration

## Purpose

This capture relates the front CSI image measurement of the yellow divider to
the QCar's measured physical clearance from that divider. It does **not**
calibrate the camera lens, move the vehicle, or enable camera steering.

The calibration is needed because the yellow divider remains visible on the
critical curve while the current white-boundary estimate fails. A physical
clearance label lets later analysis infer the designated lane center rather
than treating an arbitrary pixel position as the target.

## Safety and preparation

1. Stop `run_all_cars` and confirm `vehicle_control.py` is not running.
2. Keep the QCar stationary, with its heading aligned to the lane tangent.
3. Measure horizontally from the QCar's longitudinal centerline to the nearest
   edge of the painted yellow divider, in millimetres.
4. Record whether that divider is on the vehicle's `left` or `right`.
5. Do not run YOLO or the intersection coordinator for this capture.

## Required poses

Use one session name for all poses, for example `q151_2026-09-28`.

| Pose label | Road section | Placement |
|---|---|---|
| `straight_centered` | `straight` | Physically centered in the lane |
| `straight_toward_yellow` | `straight` | Known safe offset toward yellow |
| `straight_away_yellow` | `straight` | Known safe offset away from yellow |
| `curve_0_2_centered` | `curve_0_2` | Centered and tangent to the curve |
| `curve_2_4_centered` | `curve_2_4` | Centered at the critical lower-right curve |
| `curve_2_4_toward_yellow` | `curve_2_4` | Known safe offset toward yellow |
| `curve_2_4_away_yellow` | `curve_2_4` | Known safe offset away from yellow |

Centered means the vehicle body/wheel envelope is centered between the
physical lane boundaries—not merely that the camera image looks centered.

## Run one stationary capture from the local PC

Run this from the project directory. The wrapper uses the existing local-only
credentials and `REMOTE_PATH` in `config.txt`. Replace the labels, clearance,
and side for the current pose:

```powershell
.\capture_lane_calibration.bat `
  --session q151_2026-09-28 `
  --pose curve_2_4_centered `
  --road_section curve_2_4 `
  --yellow_boundary_side left `
  --vehicle_center_to_yellow_mm 127 `
  --duration_s 10
```

The launcher refuses to run if `vehicle_control.py` or another CSI lane-camera
process is active. It uploads only the camera measurement dependencies and
never launches the driving controller. The process exits after 10 seconds.
Expected remote outputs are one uniquely named CSV and raw/annotated frame
folders. Repeat the command for each pose. The CSV records the pose label,
section, boundary side, centerline-to-divider distance, and yellow-line image
measurements on every row.

## Acceptance before steering integration

The captures must first show that:

- the yellow boundary is detected consistently at the straight and both
  curves;
- image measurements change monotonically when the vehicle is moved toward or
  away from the divider;
- repeated centered poses give similar measurements;
- a held-out pose can predict centerline-to-divider distance within an agreed
  tolerance.

Camera steering remains disabled until these checks pass. If they fail, the
next action is detector/geometry improvement—not controller-gain tuning.
