# Car A: Current Setup

How car A (`192.168.0.151`) is set up **now**: settings, room requirements, how to run it, how to check it, and what's next. This is the doc to keep up to date. How each setting was found is in the [history records](#8-history).

**Status (2026-10-28):** car A drives the route `0 → 2 → 4 → 14 → 16 → 18 → 12 → 0` and stays in its lane. **Verified at `v_ref = 0.2` and `0.4`.** The 0.4 run looked the same as 0.2, just faster: full lap, 0 GPS readings rejected, `cross_track` within −4 … +6 cm, average size 2.5 cm (1.4 cm at 0.2). The old loop `0 → 2 → 4 → 6 → 0` was verified at 0.2 and 0.4 with the earlier settings.

**Update (2026-11-18):** **camera steering (step C3, `use_camera_steer = True`) drove car A from the straight before node 0 through 0 → 2 → 4 to just before node 14** (then a planned stop), at 0.2 m/s in four drives. The GPS gives only the position along the route. **2026-11-19: place-specific code removed** (planned stop, start zone before node 0, `cs_edge_ignore`, `gps_corrections` table; camera report §42–§43): the car can start anywhere on the route and drives on until a safety stop. Not yet driven. The 0 → 2 curve is bridged mostly by dead reckoning (the front camera can't measure it reliably). Now set to **0.4 m/s** for the next drives. The localization still fails on 4 → 14, but camera steering carries on past it. See the [camera lane correction report](history/2026-11-10_camera_lane_correction.md) (top = where to pick up). For a drive on the GPS instead, set `use_camera_steer = False` (no correction table any more).

---

## 1. Current configuration

All values are in [qcar/vehicle_control.py](../qcar/vehicle_control.py).

| Setting | Value | What it does |
|---|---|---|
| `nodeSequence` | `[0, 2, 4, 14, 16, 18, 12, 0]` | Fixed route. Old loop `[0, 2, 4, 6, 0]` and the original random node selection are kept as comments |
| `v_ref` | `0.4` m/s | Target speed. GPS + table steering verified at 0.2 and 0.4 (2026-10-28). Camera steering: four drives at 0.2 (2026-11-18), now 0.4 (camera report §41); 0.2 is commented out next to it |
| `K_stanley` | `1.5` | Stanley steering gain |
| `steering_trim` | `-0.04` rad | Straight-ahead correction for car A's steering servo; added only in `qcar.write(u, delta + steering_trim)`. **Changed 2026-11-19 from −0.14** (camera report §47: −0.14 was ~6° too far right). Revert: `-0.14` |
| `K_soft` | `0.05` m/s | Stanley softening: cross-track term atan(K·e/(v + K_soft)), finite near standstill (2026-11-19, §47). Revert: `0.0` |
| `cs_dr_model` | `'axle'` | Camera steering dead reckoning (2026-11-20, camera report §53–§57): rear axle carried along the body heading, front axle = rear + wheelbase (`lane_fusion.axle_step`, uploaded with the other `qcar/*.py`). The old model steered the rear axle onto the lane on arcs (front 4–5 cm outward). `cs_enc_axle = 'front'`. Revert: `'old'` |
| `cs_shift_use` | `True` | Along-route correction from the camera at curve starts/ends (§55–§57): `lane_camera.py` sends the best along-route shift of the lane shape. With the fusion filter it is a weighted measurement of s (sd 4 / 7 cm by fit quality); with `cs_estimator = 'c3'` one usable frame moves s since §68 (`cs_shift_confirm = False`; `True` = the earlier two frames agreeing within 6 cm). Fixes the late turn-in at the roundabout (~16 cm). Revert: `False`, or `lane_camera.py --no-shift` |
| `cs_estimator` | `'c3'` | **Back to the C3 estimator since §74** (drive H on the filter stopped in the roundabout like F, §73; drive G's setup, full lap). Before: **the fusion filter §72–§73** (it was off §64–§71 after drive F; now passes 11/12 recorded drives in `replay_check.py`). Speed drops from `v_ref` to `cs_fus_v_min` 0.2 m/s as the offset uncertainty grows from 1.5 to 4 cm; stop above `cs_fus_sd_stop` 7 cm; blind-distance backstop `cs_fus_dr_max` 3.5 m. Revert: `'c3'` (drive G's setup). Camera steering estimator (2026-11-20, §61–§62): `lane_fusion.LaneFusion` Kalman filter on s, offset, body heading and GPS bias: encoder + gyro (axle model) prediction, camera fits, camera along-route measurements and **every** GPS reading (x, y, heading), each tested against its uncertainty. Replaces the jump gate / confirmation / re-lock rules; safety stops unchanged. Offline: re-acquisition error 8.3 → 4.0 cm on 8 drives. Revert: `'c3'` |
| `use_curve_speed` | `True` | (Left on since drive B, 2026-11-20; drive B showed no gain in the curves, §51; user's choice.) Camera steering only: speed target min(`v_ref`, √(`v_curve_alat`/k)) for the largest curvature from L behind to `v_curve_ahead` 0.5 m ahead; `v_curve_alat` 0.13 m/s², `v_curve_min` 0.2 m/s (straights 0.40, 0 → 2 0.34, roundabout 0.30, 16 → 18 0.28 m/s). 2026-11-19, §47 |
| Tracked point | `0.1` m ahead of the pose | The point the Stanley controller keeps on the path (`p = ... * 0.1`) |
| `gps_scale` | `1.0` | GPS x/y are divided by this first. 1.035 fitted the old calibration (kept as a comment) |
| `use_camera` | `False` | The 2026-11-11 camera-bias steering (below) is off: it crossed the divider on 0 → 2. Replaced by `use_camera_steer` |
| `use_camera_steer` | `True` | **Camera steering (step C3, camera report §35, 2026-11-18).** `True` = steer from the camera's offset/heading to the lane; the GPS gives only the position along the route. Settings `enc_scale` 0.98, `gps_front_ahead` 0.11 m, `cs_*` (safety stops: `cs_dr_max` 2.3 m without a camera fit, `cs_max_offset` 0.12 m, `cs_cam_timeout` 0.5 s; no planned stop since 2026-11-19). Start: anywhere on the route, within `cs_start_lat` 0.15 m of it by GPS, pointing along it (`cs_start_head` 90°). Procedure: §3, "First camera-steering drive" |
| `log_odometry` | `True` | Logging only: encoder, tachometer, gyro, EKF pose and steering command to `odometry_drive.csv` on the car every 0.02 s (2026-11-18). Same first columns as `odometry_log.py`'s `odometry.csv` |
| `cam_tau` / `cam_min_conf` / `cam_max_age` | `0.3` s / `0.6` / `0.2` s | Bias smoothing time; camera trusted only in `both` mode, at this confidence or more, with a result this fresh |
| `cam_max_disagree` / `cam_max_bias` | `0.10` m / `0.15` m | Watchdog: camera ignored (bias held) if it differs from GPS + bias by more than this once driving; bias capped at this size |
| `gps_max_jump` / `gps_max_heading_jump` | `0.3` m / 30° | A GPS reading further than this from the EKF estimate is ignored |
| `gps_timeout` | `2.0` s | If no GPS reading is accepted for this long, the car stops and the script ends |
| `calibrationPose` | `[0, 2, -π/2]` | Where the car must sit when calibrating |

`steering_trim` and `gps_scale` were measured on **car A only**.

### GPS correction table (removed 2026-11-19)

The position-dependent `gps_corrections` table, `gps_correction_radius` and `use_gps_table` were removed (camera report §43; the old rows are kept there). `correct_gps()` now applies only `gps_scale`. GPS-only steering (`use_camera_steer = False`) therefore runs on the uncorrected GPS, which is several cm off on the lower part of the map.

### Camera lane correction (2026-11-11, plan step 5)

`lane_camera.py` sends every result to `127.0.0.1:18555`. Each control loop, `vehicle_control.py`:
1. Computes the GPS offset of the tracked point from the current path segment (`path_left_offset()`, + = left, the camera's sign).
2. If the camera is trusted, moves `cam_bias` towards **camera offset − GPS offset** (low-pass, `cam_tau`). Otherwise it holds `cam_bias`. This includes curves and intersections, where the camera reports `curve`/`none`.
3. Steers from the tracked point shifted sideways by `cam_bias`, so on straights Stanley's cross-track equals the camera's offset.

The bias is also learned **while parked** during `startDelay` (1 s; the watchdog is off then, so a large start bias is allowed in). The car therefore starts from the camera's view of the lane, not from the GPS. If `lane_camera.py` isn't running, the bias stays 0 and the car drives on raw GPS (`CAMERA: no data` in the log).

### Code changes compared with the Quanser original

| File | Change |
|---|---|
| [qcar/vehicle_control.py](../qcar/vehicle_control.py) | Settings above. `correct_gps()` (`gps_scale` only since 2026-11-19) applied to GPS x/y in two places (start position and EKF update). EKF started from the first GPS reading. GPS-jump check and stop. `remove_backward_points()` on the route and the approach path. `steering_trim` in `qcar.write`. Debug prints (§5). Camera lane correction: `CameraReceiver`, `path_left_offset()`, `use_camera` (2026-11-11). Camera steering `use_camera_steer` (2026-11-18) |
| [qcar/lane_camera.py](../qcar/lane_camera.py) | New (2026-11-10): front camera lane offset, CSV log and overlay images. Sends each result over UDP (`--udp-port`, default 18555, 0 = off) since 2026-11-11 |
| [python/start.py](../python/start.py) | `vehicle_control.py` output saved to `vehicle_control.log` on the car |
| [tools/on_car/check_pose.py](../tools/on_car/check_pose.py) | New: static localization check (§5). `nodeSequence` set to the new route |

**`remove_backward_points()`:** Quanser's `generate_path` sometimes overshoots a node by a few mm, which leaves a tiny backward step in the path. The steering controller can never get past one (it made the car miss node 18). The function drops these points for any route: 38 of the 552 node-to-node paths on the map have one. See the [new-route record](history/2026-10-28_new_route_investigation.md) §11.

---

## 2. Room layout requirements

The car's position comes from lidar scans matched against what was recorded at calibration, so **the room is part of the setup**.

- **Keep in place:** the white walls near the node 14 side, the landmark near node 14, and the landmark beside the top road (between the top-right corner and the circle). Without them, the upper right lane and top road were confused with other places on the map.
- **Lower part of the map** (nodes 0, 2, 4): no walls nearby. The position there is off by several cm and wanders ±2–3 cm, which the correction table used to compensate for (removed 2026-11-19; camera steering doesn't use the GPS sideways position).
- **If anything around the mat moves**, recalibrate and then do everything in [After a recalibration](#after-a-recalibration).

### After a recalibration

Measured offsets belong to one calibration. After recalibrating:
1. Survey the route with `check_pose.py` (§5), including the upper right lane, the top road and the lower part.
2. Check whether `gps_scale` still fits (it is 1.0 for the current calibration).
3. Test drive at `v_ref = 0.2` first.

---

## 3. How to run car A

1. **Calibrate** (only after the room, mat or lidar setup changes, or if the car drives noticeably off):
   - Place car A at **(0, 2) facing −90°** (pointing down the map, just above node 11). Mark this spot and direction with tape and line the car up exactly; small placement errors here grow across the map.
   - On the car over SSH: `python vehicle_control.py -c 1`. The script exits once calibration is done.
   - Calibration is stored **on that car**. Calibrating one car does nothing for another.
   - Then see [After a recalibration](#after-a-recalibration).
2. **Place the car at the start:** in node 0's lane, about **50 cm before the node 0 marker**, facing down the lane (−90°). The GPS reads about 0.2 m further along the lane than the mat marker, so 50 cm on the mat is about 30 cm before node 0 in map coordinates. If the car is placed on or past node 0, it first drives a full approach lap (the old loop) before starting the route.
3. **Start the camera first** (when `use_camera = True`), in its own SSH window. `start.py` uploads `qcar/*.py` only when the cars are launched, so copy a changed `lane_camera.py` by hand first:
   ```
   scp qcar/lane_camera.py nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/
   ssh nvidia@192.168.0.151
   cd /home/nvidia/Documents/self_driving/multi_vehicle_self_driving
   python -u lane_camera.py
   ```
   Wait until it prints `both` with an offset near 0 for the parked car, then launch. Stop it with Ctrl+C after the run, and copy `lane_camera.csv` and `lane_debug/` along with the log. Empty the PC's `logs/car A/lane_debug/` first, so images from different runs don't mix.
4. **Start the drive:** **save all files first**, then run `run_all_cars.bat` on the PC. It reads [config.txt](../config.txt) and calls [python/start.py](../python/start.py), which uploads every `qcar/*.py` file to the car at launch. A change saved after launching is not used in that run.
5. **Check the placement** (optional but recommended), from a second terminal on the PC:
   ```
   ssh nvidia@192.168.0.151
   tail -f /home/nvidia/Documents/self_driving/multi_vehicle_self_driving/vehicle_control.log
   ```
   The first `GPS:` line, about 10 s after launch, should read **y ≈ 0.3 to 0.5**. If it is below about 0.25, stop and move the car back.
6. **Stop:** `stop_all_cars.bat`. The car also stops itself if the GPS-jump check accepts no reading for 2 s (`STOPPED:` in the log).
7. **Copy the log to the PC** if you want to keep it. Each run overwrites it on the car, so give each copy its own name:
   ```
   scp nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/vehicle_control.log "logs/car A/vehicle_control_<name>.log"
   ```
   Also copy `odometry_drive.csv` (overwritten each run too).

### Measure-only camera recording drive (camera report §32, step before C3)

GPS steering (`use_camera = False`, `use_camera_steer = False`; no correction table since 2026-11-19); the camera only records.
1. Copy the new `lane_camera.py` to the car (checkerboard calibration since 2026-11-18), as in step 3.
2. **Place the car parked on the straight before node 0, with the camera seeing both lines** (as in step 2, ~50 cm before the node 0 marker). At node 0 itself the curve is in view and the camera reads `curve`.
3. Start the camera first: `python -u lane_camera.py --save-raw --udp-port 0`. Wait for `both` with an offset near 0.
4. Launch with `run_all_cars.bat`. Keep the car still until it moves off: the gyro bias is measured during the first second.
5. Stop at node 4 or wherever it stops (the 4 → 14 GPS failure, §6 of the report).
6. Copy back `vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv` and the new `raw_frames/<start time>/` folder.

### First camera-steering drive (step C3, camera report §35)

1. In `vehicle_control.py`: `use_camera_steer = True` (leave `use_camera = False`). There is no planned stop any more (`cs_stop_s` removed 2026-11-19): the car drives on past node 14, where the method is untested, until a safety stop or `stop_all_cars.bat`.
2. Copy the new `lane_camera.py` to the car (it now does the map-shape fit).
3. **Place the car parked anywhere on the route**, centred in its lane and pointing the way the route goes, preferably on a straight where the camera sees both lines (it takes its starting offset and heading from the camera while parked; on a curve or in an intersection it may have none). The script refuses to start if the GPS puts the car more than 15 cm from the route (`CAMSTEER: not starting: …`). The straight ~50 cm before the node 0 marker is the spot all drives so far started from.
4. Start the camera first, **with UDP on** (no `--udp-port 0`): `python -u lane_camera.py --save-raw`. Wait for `both` with an offset near 0.
5. Launch with `run_all_cars.bat`. Keep the car still for the first second (gyro bias). Have `stop_all_cars.bat` ready, and stop by hand if a wheel touches a line.
6. Expected console lines: `CAMSTEER: start at s = … m along the route (front axle, route length … m)` (the straight before node 0 is now near the end of the route, s ≈ 13.5), then `pose= … cs: s=… offset=… heading=… src=E/YE/DR …` twice per second, and finally a `STOPPED (camera steering): …` safety-stop reason (or a hand stop). `lane_camera.py` prints `map-fit …` once per second.
7. Copy back `vehicle_control.log`, `odometry_drive.csv` (now with `cs_s`, `cs_d_cm`, `cs_h_deg`, `cs_src`), `lane_camera.csv` (with `mf_*`), `lane_debug/` and the `raw_frames/` folder.

**Since 2026-11-20 (camera report §57):** copy the new `lane_camera.py` to the car by hand again (it now runs the along-route shift fit, `sh_*` columns in `lane_camera.csv`; `--no-shift` turns it off). `vehicle_control.py` imports `lane_fusion.py`, which `start.py` uploads with the other `qcar/*.py`. New console lines: `lane_camera.py` adds `shift +0.18 m` to its `map-fit` line when a shift is found; `vehicle_control.py` prints `CAMSTEER: along-route correction +18.0 cm from the camera …` when it applies one (expected once, just before the roundabout), and the `pose=` lines show `shift=<count>`. After the drive: `python tools/checks/steer_check.py <folder>` and `python tools/checks/curve_shift_check.py "<name>|<folder>/<raw frames>|<folder>/odometry_drive.csv"` (the roundabout entry shift should now be ~0).

**Since 2026-11-20 evening (camera report §62; on only §62–§63 and §72–§73, off since §74):** with `cs_estimator = 'fusion'` the fusion filter steers. The `pose=` lines' `v_target` shows the uncertainty slow-down; a stop for uncertainty prints `STOPPED (camera steering): offset uncertainty … cm`. Copy the new `lane_camera.py` to the car again (one more UDP field, `sh_rms`). Console: `CAMSTEER: estimator = fusion filter (dead reckoning axle, encoder = front axle)` at the start; `CAMSTEER: camera along-route measurement +… cm -> s … -> … m` at curve starts/ends (expected before the roundabout and around its exit / 16 → 18); the `pose=` lines add `gps_rej=`, `sd_d=` (offset uncertainty) and `sd_s=` (along-route uncertainty). After the drive also run `python tools/checks/shift_meas.py <frames> <odometry_drive.csv>` and `python tools/replay/fusion_replay.py … --dr axle --enc front --shift <folder>/shift_meas.csv`.

---

## 4. Tuning options and trade-offs

### Tracked point (`* 0.1` on the `p = …` line)
| Value | Trade-off |
|---|---|
| 0.2 (original) | Front of the car follows the path closely; body and rear cut the inside of curves |
| **0.1** (current) | Body stays in lane; front swings slightly wider; the inside rear wheel is the part that gets close to the divider |

### `K_stanley`
Higher values correct sideways error harder. Too high causes zig-zagging on straights. 1.5 showed no wobble at 0.2–0.4 m/s. The correction gets stronger as speed drops (the term is `K · error / speed`), so retune if `v_ref` changes a lot.

### `v_ref`
On the new route, 0.4 behaved like 0.2 (average `cross_track` 2.5 vs 1.4 cm). On the old loop, 0.2 and 0.4 gave the same tracking. 0.75 was too fast for the curves with the original settings and hasn't been retested.

### GPS-jump check
`gps_max_jump = 0.3` m is about 4 GPS readings' worth of travel at 0.4 m/s (readings arrive at about 5 Hz) plus jitter. Raising `v_ref` a lot may need a larger value. `gps_timeout = 2` s means up to about 0.8 m of driving on odometry alone at 0.4 m/s before the car stops.

---

## 5. Diagnostic tools

### `vehicle_control.log`
Lines:
- `GPS: (x, y, heading)`: every **accepted** GPS reading, after `correct_gps()`.
- `GPS rejected: (x, y, heading) jump=… heading_jump=…`: a reading the GPS-jump check ignored.
- `STOPPED: no GPS reading accepted for 2 s. Last estimate (…)`: the car stopped itself.
- `pose=… path_pt=… cross_track=… t_epoch=…`: twice per second while on the route. `cross_track` is the sideways distance (m) between the tracked point and the path. **Negative = car left of the path.** `t_epoch` (added 2026-11-10) is the car's clock time, the same as the `t_epoch` column in `lane_camera.csv`, so the two logs can be matched. Note the opposite sign: the camera's `offset_cm` is **positive** when the car is left of the lane centre.
  With `use_camera = True` the line also has `cam=<mode>/<offset>cm  bias=<cm>  used=<0/1>` (before `t_epoch`). `used=1` means the camera updated the bias in that loop. **`cross_track` is then measured from the shifted point**, so it already includes the bias. On a trusted straight it equals −camera offset. The GPS-only offset (+ = left) is −`cross_track` − `bias`.
- `CAMERA: bias at start …` / `CAMERA: no data …`: once, when the car starts moving. Shows the bias learned while parked, or that `lane_camera.py` wasn't running.

> **Important:** `cross_track` is measured against the car's *own estimate*. It cannot detect localization errors: the estimate and the real car move together. A log can show 1 cm tracking while the real car is on the divider. Always compare with what the car is physically doing.

Useful patterns:
- Long run of `GPS:` lines before any `pose=` line: the car is driving the approach lap, not the route (placed on or past node 0).
- `cross_track` consistently one sign: a constant bias (steering trim or heading).
- `cross_track` small while the car is visibly off: localization offset; use `check_pose.py`.
- `path_pt` running off in a straight line while the car should turn: a path problem (e.g. a backward step; `remove_backward_points()` should prevent this).
- `GPS rejected:` lines in a row: localization lost at that spot; check that part of the room.

### `check_pose.py`
[tools/on_car/check_pose.py](../tools/on_car/check_pose.py) reads only the GPS (motors are not used) and prints, averaged over the last 25 readings (~5 s):
- `side_offset`: sideways distance of the **raw** reading from the route path. **+ = left of the path (toward the divider), − = right (toward the outer edge).** The controller moves the real car the opposite way, so a negative reading puts the real car toward the divider.
- `spread`: min..max over the window, i.e. jitter at rest.
- `heading_err`: car heading minus path direction.
- `n`: window fill. It resets when a reading is more than 10 cm from the window's average (the car was moved, or a stray reading); read values only at `n=25/25`.

`gps_scale` is **not** applied in this script. `nodeSequence` and `calibrationPose` at the top must match `vehicle_control.py`.

**Survey procedure:**
1. **Place the car:** its **middle** (the lidar) on the spot, centred in the lane and pointing the way it drives on the route. Node markers are the easiest spots.
2. **Read:** wait for `n=25/25`, then watch for another 5 s.
3. **Note anything wrong:** a `GPS=` position that isn't where the car is, or `n` dropping back while the car is still.
4. **Move to the next spot by rolling the car, not lifting it.** A lifted car can lose the localization.
5. **Stay out of the lidar's view** (below about car-roof height) while each spot settles.
6. Measure only at spots *after* the route's first node: before node 0, the nearest path point is the end of the loop.

Commands. On the PC:
```
scp tools/on_car/check_pose.py nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/
ssh nvidia@192.168.0.151
```
On the car:
```
cd /home/nvidia/Documents/self_driving/multi_vehicle_self_driving
python -u check_pose.py | tee -i check_pose.log
```
Press **Ctrl+C** after the last spot (`-i` keeps `tee` running so the last lines are saved), then `exit`. Back on the PC:
```
scp nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/check_pose.log "logs/car A/check_pose/check_pose_<round>.log"
```

---

## 6. Known limitations

- **Tight lane clearance.** Lanes are ~27 cm wide and the car is ~19 cm wide, leaving ~4 cm per side. Tracking error (~1.5 cm) plus GPS wander (±2–3 cm on the lower part) use most of it, so occasional contact with a lane line is expected with GPS-only lane keeping.
- **Localization depends on the room.** Moving walls or landmarks breaks it (§2).
- **GPS-only steering has no correction on the lower part** since the table was removed (2026-11-19): it is several cm off there, plus ±2–3 cm wander. Camera steering (`use_camera_steer`) doesn't use the GPS sideways position.
- **Along-lane offset.** The GPS reads about 0.2 m further along the lane than the mat markers. This doesn't affect lane keeping, but matters for start placement (§3).
- **Stray readings.** Single wrong GPS readings appear, mostly while the car is moved by hand. The GPS-jump check ignores them while driving. A loss lasting more than 2 s stops the car rather than letting it steer off.
- **Startup pause.** The car rolls a few cm, pauses ~2 s, then continues, at the same spot every run. Most likely YOLO detecting something near the start (`vGain` drops). Harmless.
- **Argument types.** `-c` and `-n` in `vehicle_control.py` are read as strings: `-n 0` selects the *second* node set and `-c False` counts as true. Use `-c 1` to calibrate and omit `-c` otherwise. `-n` has no effect, since `nodeSequence` is hard-coded.
- **Debug output.** The `GPS:` print fills the log; it can be removed or rate-limited once tuning is finished.

---

## 7. Next steps

1. **Camera lane correction:** use camera lane detection to correct the remaining few cm of sideways error against the painted lines, with the map-based controller still handling the route. This is the way past the GPS accuracy limit; it replaced the correction table (removed 2026-11-19). Plan: [plans/camera_lane_correction.md](plans/camera_lane_correction.md) (builds on skill activity 06, [skills_activities/06-lane_keeping/](../skills_activities/06-lane_keeping/)).
2. **Per-car settings.** Move `steering_trim` and `gps_scale` out of the code:
   - `steering_trim` and `gps_scale` as per-car lists in `config.txt`, in the same order as `QCAR_IPS`, passed through `run_all_cars.bat` → `start.py` → `vehicle_control.py` arguments.
   - Add `type=float`/`type=int` on all arguments. This also fixes the `-c`/`-n` issue (§6).

   Required before car A and car B run **together** with different values. Car B's drive of 2026-11-20 (camera report §61) showed car A's `steering_trim` and camera calibration fit car B, so no value differs yet; `enc_scale` may (car B k_gps 1.026 vs 0.994, one drive).
3. **Calibrating a new car.** Repeat the car A procedure:
   1. Calibrate.
   2. Start ~50 cm before the first node's marker.
   3. Run with `steering_trim = 0` and read the average `cross_track`.
   4. Set `steering_trim ≈ average × K_stanley / v_ref` (a negative average gives a negative trim; car A: −0.037 × 1.5 / 0.4 ≈ −0.14). **Better (camera report §47):** with camera steering, read the commanded `delta` on a straight in `odometry_drive.csv` (and the achieved angle atan(L · gyro / speed)); the trim should make the command ≈ 0 there. Car A: −0.04.
   5. Survey with `check_pose.py` and check `gps_scale` (§2).
4. **Optional: walls near the lower part** of the map, to reduce the wander. This needs a recalibration and a full re-survey (§2).
5. **Multi-vehicle.** Once the above are in place: separate routes per car, then coordination at shared intersections.

**Done** (from the earlier next-steps list):
- **GPS-jump safety stop** (2026-10-28): readings further from the estimate than the car could have moved are ignored, and the car stops if none are accepted for 2 s (§1, §5). Details in the [new-route record](history/2026-10-28_new_route_investigation.md) §9.

---

## 8. History

How the current settings were found. These are finished records, kept as written; settings in them may be out of date.

| Record | What it covers |
|---|---|
| [history/2026-10-26_old_loop_tuning.md](history/2026-10-26_old_loop_tuning.md) | Old loop `0 → 2 → 4 → 6 → 0`: log setup, approach-lap issue, `steering_trim`, `gps_scale = 1.035`, tracked point |
| [history/2026-10-28_new_route_investigation.md](history/2026-10-28_new_route_investigation.md) | New route: localization survey rounds 3–9, walls and landmarks, GPS-jump check, node 16 backward step, correction table, drives 1–6 |
| [history/2026-11-10_camera_lane_correction.md](history/2026-11-10_camera_lane_correction.md) | **Camera lane correction, one combined report (from 2026-11-10, ongoing).** Steps 0–2 (camera choice, calibration, `lane_camera.py`), laps A/B (room change), recalibration and the 4 → 14 localization failure, ruler check (step 4), camera steering (step 5) and its first drive (crossed the divider on the 0 → 2 curve), curve method discussion. **§13 = where to pick up** |
