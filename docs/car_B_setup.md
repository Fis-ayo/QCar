# Car B: Current Setup

How car B (`192.168.0.152`) is set up **now**: settings, how to run it, how to check it, and what's next. This is the doc to keep up to date. How each setting was found is in the [history records](#7-history).

**Status (2026-11-20): car B runs car A's current code and settings unchanged, and needs no car-specific tuning so far.** First camera-steering drive (drive A, §4): at 0.4 m/s from the straight before node 0 through 0 → 2 → 4 → 14 and the roundabout, in its lane, with steering, camera and lane offsets that match car A's (straights: command +0.2°, offset +1.4 cm; node 2 first fit −2.4 cm). It then crossed into the other lane at the **16 → 18 S-bend near node 18**, the same failure as car A's drive D. Cause: the along-route position, not car B's settings (§4). Car B's encoder may read ~3 % differently from car A's (one drive, §4): to confirm from the next drives, a possible small per-car `enc_scale`, not major tuning.

The October 2026 setup (GPS steering with car A's correction table, first full lap 2026-10-29) is in the [first-runs record](history/2026-10-29_car_B_first_runs.md).

---

## 1. Current configuration

Car B runs the same [qcar/vehicle_control.py](../qcar/vehicle_control.py) and [qcar/lane_camera.py](../qcar/lane_camera.py) as car A, so all settings are **car A's values**. See [car_A_setup.md §1](car_A_setup.md#1-current-configuration) for what each one does.

| Setting | Value | Checked on car B? (drive A, 2026-11-20) |
|---|---|---|
| `nodeSequence` | `[0, 2, 4, 14, 16, 18, 12, 0]` | n/a (route) |
| `use_camera_steer` | `True` | Yes: camera steering works on car B |
| `v_ref` / `use_curve_speed` | `0.4` m/s / `True` | Yes: mean 0.34 m/s moving, no problem in the curves |
| `K_stanley` / `K_soft` | `1.5` / `0.05` | Yes: no wobble, never at the steering limit (longest at the limit 0.03 s) |
| `steering_trim` | `-0.04` rad | **Yes, fits.** Command on the 4 → 14 straight +0.2…+0.4° (car A +0.0…+0.2°): within ~0.3° (0.005 rad). The October estimate for car B (≈ −0.06) was for the old GPS steering |
| Camera calibration in `lane_camera.py` | car A's (intrinsics, pitch, height 10.5 cm, `CAM_AHEAD`) | **Yes, fits.** Lane width on straights 25.3 cm (true 25.7, car A 26.6), offset +0.8 cm |
| `cs_dr_model` / `cs_shift_use` | `'axle'` / `True` | Running; the along-route correction did not fire (§4) |
| `enc_scale` | `0.98` | **Open.** k_gps = 1.026 on car B vs 0.994 ± 0.013 on car A (§4); one drive only |
| `gps_scale` / `gps_max_jump` / `gps_timeout` | `1.0` / `0.3` m / `2.0` s | Shared; same 4 → 14 GPS failure as car A |
| `calibrationPose` | `[0, 2, -π/2]` | Same spot as car A |

**To run car B,** [config.txt](../config.txt) must point at it: `QCAR_IPS=[192.168.0.152]`, `CALIBRATE_IP` and `PROBING_IP` = `192.168.0.152` (`LOCAL_IP=192.168.0.153` is the PC).

**Both cars share one `vehicle_control.py`** (per-car settings not done yet, [car A next step 2](car_A_setup.md#7-next-steps)). Not a problem while they drive one at a time with the same values; needed before they run together if any value differs (first candidate: `enc_scale`).

### Car B software difference: Quanser libraries

Car B's original Quanser Python libraries were an older version. Their `SDCSRoadMap` (`hal.products.mats`) had no `initial_check`, so `vehicle_control.py` crashed at start-up (`AttributeError: 'SDCSRoadMap' object has no attribute 'initial_check'`).

**Fix:** car A's library contents (`hal`, `pal`) were copied into car B's library folder. The folder kept **car B's own name**, because car B's Python search path points at that name.

Check on the car (should print `True`):
```
python3 -c "from hal.products.mats import SDCSRoadMap; print(hasattr(SDCSRoadMap, 'initial_check'))"
```

---

## 2. Room layout requirements

Same room and mat as car A: see [car_A_setup.md §2](car_A_setup.md#2-room-layout-requirements). Calibration is stored on each car, so **car B needs its own calibration** after any change to the room, the mat or its lidar: car B at **(0, 2) facing −90°**, then on the car `python vehicle_control.py -c 1`.

---

## 3. How to run car B

Same procedure as car A's camera-steering drive, [car_A_setup.md §3 "First camera-steering drive"](car_A_setup.md#first-camera-steering-drive-step-c3-camera-report-35), with car B's IP:

1. **Point [config.txt](../config.txt) at car B** (§1) and **save all files**: `start.py` uploads every `qcar/*.py` at launch.
2. **Copy `lane_camera.py` to the car by hand** (the camera is started before the launch):
   ```
   scp qcar/lane_camera.py nvidia@192.168.0.152:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/
   ```
3. **Place the car** parked on the straight ~50 cm before the node 0 marker, centred, pointing down the lane.
4. **Start the camera** in its own SSH window, UDP on:
   ```
   ssh nvidia@192.168.0.152
   cd /home/nvidia/Documents/self_driving/multi_vehicle_self_driving
   python -u lane_camera.py --save-raw
   ```
   Wait for `both` with an offset near 0.
5. **Launch** with `run_all_cars.bat`. Keep the car still for the first second (gyro bias). Have `stop_all_cars.bat` ready.
6. **Copy back** into a new folder of its own, e.g. `logs/car B/<date>_<name>/`: `vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv`, `lane_debug/` and the new `raw_frames/<start time>/` folder (each run overwrites them on the car).
7. **Check:** `python tools/checks/steer_check.py "logs/car B/<folder>"`.

---

## 4. Latest run: drive A (2026-11-20), first camera-steering drive

Logs: [logs/car B/20261120_driveA/](../logs/car%20B/20261120_driveA/) (raw frames `20261120_220828`). Code and settings exactly as car A's drive E. Full analysis: [camera report §65](history/2026-11-10_camera_lane_correction.md#65-car-b-drive-a-2026-11-20-car-as-code-on-car-b-s-bend-failure).

**Result:** 10.9 m (s 13.56 → 24.45), start before node 0 → 0 → 2 → 4 → 14 → roundabout → 16 → 18 S-bend, where the car **turned into the other lane** (user: "turned too early or something"); safety stop `no camera fit for 2.30 m`.

| Stretch | Command / achieved / needed (°) | Camera-steering offset `cs_d` (cm) |
|---|---|---|
| approach, node 0 | +0.2…+0.8 / −0.2…+0.6 / 0 | +0.2, −0.4 |
| 0 → 2 arc | +14.8 / +13.5 / +16.7 | −0.8 (first fit at node 2: −2.4; car A −6…−9) |
| 2 → 4 | +12.4 / +12.1 / +16.7 | +1.1 |
| 4 → 14 straight | +0.2 / +0.2 / 0 | +1.4 |
| roundabout | +20.1 / +18.5 / +22.2 | 0.0 (mostly dead reckoning) |

- **Settings:** every stretch matches car A's drives D/E within about 1° and 1 cm. Trim and camera calibration fit car B (§1).
- **Why it left the lane at the S-bend:** the GPS failed on 4 → 14 (from s ≈ 20.0, every reading rejected, as on car A), so the position along the route came from the encoder only. The camera's along-route correction at the roundabout entry needs two agreeing frames: car B got **one** good frame (+24 cm) and the next one failed, so **no correction was applied** (car A drive E: +25 cm, drive D: +19 cm). The car reached the S-bend with its along-route position behind, kept the roundabout's left turn too long and switched to the right turn too late. Same mechanism as car A's drive D: a code/estimation issue shared by both cars, not car B tuning.
- **Encoder (`tools/checks/scale_check.py` method, camera report §60):** k_gps (map m per raw encoder m, start → s 4.2) = **1.026** on car B vs 0.994 ± 0.013 over car A's seven drives; k_cam (last good GPS → painted roundabout entry) 1.005 vs 1.045 ± 0.016. One drive, and the two numbers point different ways, so **not yet a reason to change `enc_scale`**; repeat on the next car B drives.
- GPS: same repeatable failure on 4 → 14 as car A (wrong solution ~1 m to the right, then near (−0.6, 3.6) in the roundabout). Not a car B problem.

---

## 5. Known limitations

- **Along-route position without GPS** (shared with car A): after the 4 → 14 GPS failure, the along-route position comes from the encoder plus one camera correction at the roundabout entry, which needs two agreeing frames. When it doesn't fire, the S-bend at node 18 fails (car B drive A, car A drive D). Fix planned in code, for both cars (§6).
- **Calibration and the 4 → 14 lane.** Recalibrate car B after any change around the mat.
- **A manual stop ends the log with `STOPPED:`.** Stopping the cars shuts down the GPS, so the 2 s timeout prints `STOPPED: no GPS reading accepted for 2 s` even though nothing went wrong.
- **YOLO start-up message.** `YOLO Server error: status check failed` appears twice at start, followed by `Reconnected to yolo Server`. Harmless.
- All of [car A's limitations](car_A_setup.md#6-known-limitations) also apply.

---

## 6. Next steps

1. **No car B tuning needed for now.** Trim, camera calibration, controller gains and speeds carry over from car A (§1, §4).
2. **Along-route fix, in code for both cars** (camera report §58–§60): the fusion filter with the camera shift as a weighted measurement (one clean frame counts), axle model, GPS bias states. **Status 2026-11-20:** written and replayed on car A's drives (§61: re-acquisition error 8.3 → 4.0 cm), put on the cars behind `cs_estimator` (§62), stopped in the roundabout on car A's drive F (§63), so off again (§64); on again §72, **off again since §74 for both cars** (`cs_estimator = 'c3'`; car A's drive H stopped in the roundabout on it, §73): `replay_check.py` passes car B's drive A (S-bend) and drive B (blind 12 → 0 straight) with it.
3. **`enc_scale` for car B:** run `scale_check.py`'s two ratios on the next 2–3 car B drives. If k_gps stays ~3 % above car A's, car B gets its own `enc_scale`, which makes per-car settings (step 4) necessary.
4. **Per-car settings** ([car A next step 2](car_A_setup.md#7-next-steps)): before car A and car B run together, if any value differs.
5. **Repeated laps** on both cars to measure the success rate ([supervisor feedback plan](plans/supervisor_feedback_2026-11-05.md)), then multi-vehicle.

---

## 7. History

| Record | What it covers |
|---|---|
| [history/2026-10-29_car_B_first_runs.md](history/2026-10-29_car_B_first_runs.md) | Getting car A's code to run on car B: library fix, localization loss on 4 → 14, first full lap (GPS steering, October settings) |
| [history/2026-11-10_camera_lane_correction.md §65](history/2026-11-10_camera_lane_correction.md#65-car-b-drive-a-2026-11-20-car-as-code-on-car-b-s-bend-failure) | Car B's first camera-steering drive with car A's code: settings carry over, S-bend failure from the along-route position |
