# Car A: Current Setup

How car A (`192.168.0.151`) is set up **now**: settings, room requirements, how to run it, how to check it, and what's next. This is the doc to keep up to date. How each setting was found is in the [history records](#8-history).

**Status (2026-10-28):** car A drives the route `0 → 2 → 4 → 14 → 16 → 18 → 12 → 0` and stays in its lane. **Verified at `v_ref = 0.2` and `0.4`.** The 0.4 run looked the same as 0.2, just faster: full lap, 0 GPS readings rejected, `cross_track` within −4 … +6 cm, average size 2.5 cm (1.4 cm at 0.2). The old loop `0 → 2 → 4 → 6 → 0` was verified at 0.2 and 0.4 with the earlier settings.

**Update (2026-11-11):** car A was recalibrated after a room change, and the localization now fails on 4 → 14 at y ≈ 1.0–1.3 (every drive stops there). **Camera lane steering (plan step 5) is in the code:** `use_camera = True`, `use_gps_table = False` (the table is ignored, the camera's learned bias replaces it), `v_ref = 0.2`. **Driven once: straight start, but the car crossed the divider on the 0 → 2 curve** (the front camera can't measure curves yet). For a drive that isn't testing the camera, set `use_camera = False` and `use_gps_table = True` (the table is from the old room, so expect a few cm of error on the lower part). See the [camera plan](plans/camera_lane_correction.md) and the [camera lane correction report](history/2026-11-10_camera_lane_correction.md) (§13 = where to pick up).

---

## 1. Current configuration

All values are in [qcar/vehicle_control.py](../qcar/vehicle_control.py).

| Setting | Value | What it does |
|---|---|---|
| `nodeSequence` | `[0, 2, 4, 14, 16, 18, 12, 0]` | Fixed route. Old loop `[0, 2, 4, 6, 0]` and the original random node selection are kept as comments |
| `v_ref` | `0.2` m/s | Target speed. Verified on the new route at 0.2 and 0.4. Set to 0.2 for the first camera-steering tests (2026-11-11); 0.4 is commented out next to it |
| `K_stanley` | `1.5` | Stanley steering gain |
| `steering_trim` | `-0.14` rad | Cancels car A's leftward steering-servo bias; added only in `qcar.write(u, delta + steering_trim)` |
| Tracked point | `0.1` m ahead of the pose | The point the Stanley controller keeps on the path (`p = ... * 0.1`) |
| `gps_scale` | `1.0` | GPS x/y are divided by this first. 1.035 fitted the old calibration (kept as a comment) |
| `gps_corrections` | 6 rows, see below | Position-dependent GPS correction for the lower part of the map. **Measured for the old room; ignored while `use_gps_table = False`** |
| `use_gps_table` | `False` | `False` = `correct_gps()` applies only `gps_scale` (camera lane correction replaces the table, 2026-11-11) |
| `use_camera` | `True` | Camera lane correction from `lane_camera.py` over UDP (port `cam_port = 18555`). See below |
| `cam_tau` / `cam_min_conf` / `cam_max_age` | `0.3` s / `0.6` / `0.2` s | Bias smoothing time; camera trusted only in `both` mode, at this confidence or more, with a result this fresh |
| `cam_max_disagree` / `cam_max_bias` | `0.10` m / `0.15` m | Watchdog: camera ignored (bias held) if it differs from GPS + bias by more than this once driving; bias capped at this size |
| `gps_correction_radius` | `0.35` m | How far each correction row reaches before blending into the next / fading out |
| `gps_max_jump` / `gps_max_heading_jump` | `0.3` m / 30° | A GPS reading further than this from the EKF estimate is ignored |
| `gps_timeout` | `2.0` s | If no GPS reading is accepted for this long, the car stops and the script ends |
| `calibrationPose` | `[0, 2, -π/2]` | Where the car must sit when calibrating |

`steering_trim`, `gps_scale` and `gps_corrections` were measured on **car A only**, and `gps_corrections` only for the **current calibration**.

### GPS correction table

`correct_gps()` applies `gps_scale`, then adds a blend of nearby rows. Each row is `(x, y, dx, dy)`: at raw GPS position `(x, y)`, add `(dx, dy)`. Rows are weighted by `exp(−d² / 0.35²)` plus a small "no correction" weight, so the correction fades to zero about 1 m from every row. From node 14 round to node 12 it is 0.

| Row | x, y | dx, dy | Source |
|---|---|---|---|
| node 0 | 0.06, −0.06 | +0.030, 0.000 | tuned from driving (3 cm away from the divider) |
| mid 0 → 2 | 0.27, −0.81 | +0.021, +0.021 | tuned from driving (3 cm away from the divider) |
| node 2 | 1.26, −1.15 | 0.000, +0.063 | survey round 9 |
| mid 2 → 4 | 2.04, −0.88 | −0.064, +0.064 | tuned from driving (9 cm away from the divider) |
| node 4 | 2.32, 0.00 | −0.061, 0.000 | survey round 9 |
| mid 4 → 14 | 2.29, 1.43 | −0.036, 0.000 | survey round 9 |

Resulting shift of the real car away from the divider: 0 → 2 about 3 cm, 2 → 4 about 7 cm (8 cm mid-curve), 4 → 14 about 2 cm (fading out by y ≈ 2), 12 → 0 at most 3 cm just before node 0.

### Camera lane correction (2026-11-11, plan step 5)

`lane_camera.py` sends every result to `127.0.0.1:18555`. Each control loop, `vehicle_control.py`:
1. Computes the GPS offset of the tracked point from the current path segment (`path_left_offset()`, + = left, the camera's sign).
2. If the camera is trusted, moves `cam_bias` towards **camera offset − GPS offset** (low-pass, `cam_tau`). Otherwise it holds `cam_bias`. This includes curves and intersections, where the camera reports `curve`/`none`.
3. Steers from the tracked point shifted sideways by `cam_bias`, so on straights Stanley's cross-track equals the camera's offset.

The bias is also learned **while parked** during `startDelay` (1 s; the watchdog is off then, so a large start bias is allowed in). The car therefore starts from the camera's view of the lane, not from the GPS. If `lane_camera.py` isn't running, the bias stays 0 and the car drives on raw GPS (`CAMERA: no data` in the log).

### Code changes compared with the Quanser original

| File | Change |
|---|---|
| [qcar/vehicle_control.py](../qcar/vehicle_control.py) | Settings above. `correct_gps()` applied to GPS x/y in two places (start-position check and EKF update). EKF started from the first GPS reading. GPS-jump check and stop. `remove_backward_points()` on the route and the approach path. `steering_trim` in `qcar.write`. Debug prints (§5). Camera lane correction: `CameraReceiver`, `path_left_offset()`, `use_camera` / `use_gps_table` (2026-11-11) |
| [qcar/lane_camera.py](../qcar/lane_camera.py) | New (2026-11-10): front camera lane offset, CSV log and overlay images. Sends each result over UDP (`--udp-port`, default 18555, 0 = off) since 2026-11-11 |
| [python/start.py](../python/start.py) | `vehicle_control.py` output saved to `vehicle_control.log` on the car |
| [qcar/check_pose.py](../qcar/check_pose.py) | New: static localization check (§5). `nodeSequence` set to the new route |

**`remove_backward_points()`:** Quanser's `generate_path` sometimes overshoots a node by a few mm, which leaves a tiny backward step in the path. The steering controller can never get past one (it made the car miss node 18). The function drops these points for any route: 38 of the 552 node-to-node paths on the map have one. See the [new-route record](history/2026-10-28_new_route_investigation.md) §11.

---

## 2. Room layout requirements

The car's position comes from lidar scans matched against what was recorded at calibration, so **the room is part of the setup**.

- **Keep in place:** the white walls near the node 14 side, the landmark near node 14, and the landmark beside the top road (between the top-right corner and the circle). Without them, the upper right lane and top road were confused with other places on the map.
- **Lower part of the map** (nodes 0, 2, 4): no walls nearby. The position there is off by several cm and wanders ±2–3 cm, which is what the correction table compensates for.
- **If anything around the mat moves**, recalibrate and then do everything in [After a recalibration](#after-a-recalibration).

### After a recalibration

The correction table and any measured offsets belong to one calibration. After recalibrating:
1. Survey the route with `check_pose.py` (§5), including the upper right lane, the top road and the lower part.
2. Rebuild `gps_corrections` from the new lower-part readings (each row = minus the measured `side_offset`, turned into an x/y shift across the lane). Then fine-tune from driving.
3. Check whether `gps_scale` still fits (it is 1.0 for the current calibration).
4. Test drive at `v_ref = 0.2` first.

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

---

## 4. Tuning options and trade-offs

### `gps_corrections` rows
The main tuning knob for the lower part of the map. Adjust the row nearest the problem spot in **1–2 cm steps per drive**, along the lane's sideways direction:
- Car too close to the **divider** → increase the shift away from the divider.
- Car too close to the **outer edge** → decrease it.

The size of a row's shift is `√(dx² + dy²)`. The direction is the lane's left normal, i.e. pointing toward the divider: for example (+1, 0) on the x = 0 lane heading down the map, (0, +1) on the bottom lane, (−0.71, +0.71) on the 2 → 4 curve. Keep the direction and change the size. Rows only affect about 0.35–1 m around them.

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

> **Important:** `cross_track` is measured against the car's *own estimate*. It cannot detect localization errors, and with the correction table the estimate and the real car move together. A log can show 1 cm tracking while the real car is on the divider. Always compare with what the car is physically doing.

Useful patterns:
- Long run of `GPS:` lines before any `pose=` line: the car is driving the approach lap, not the route (placed on or past node 0).
- `cross_track` consistently one sign: a constant bias (steering trim or heading).
- `cross_track` small while the car is visibly off: localization offset; use `check_pose.py`.
- `path_pt` running off in a straight line while the car should turn: a path problem (e.g. a backward step; `remove_backward_points()` should prevent this).
- `GPS rejected:` lines in a row: localization lost at that spot; check that part of the room.

### `check_pose.py`
[qcar/check_pose.py](../qcar/check_pose.py) reads only the GPS (motors are not used) and prints, averaged over the last 25 readings (~5 s):
- `side_offset`: sideways distance of the **raw** reading from the route path. **+ = left of the path (toward the divider), − = right (toward the outer edge).** The controller moves the real car the opposite way, so a negative reading puts the real car toward the divider.
- `spread`: min..max over the window, i.e. jitter at rest.
- `heading_err`: car heading minus path direction.
- `n`: window fill. It resets when a reading is more than 10 cm from the window's average (the car was moved, or a stray reading); read values only at `n=25/25`.

`gps_scale` and `gps_corrections` are **not** applied in this script. `nodeSequence` and `calibrationPose` at the top must match `vehicle_control.py`.

**Survey procedure:**
1. **Place the car:** its **middle** (the lidar) on the spot, centred in the lane and pointing the way it drives on the route. Node markers are the easiest spots.
2. **Read:** wait for `n=25/25`, then watch for another 5 s.
3. **Note anything wrong:** a `GPS=` position that isn't where the car is, or `n` dropping back while the car is still.
4. **Move to the next spot by rolling the car, not lifting it.** A lifted car can lose the localization.
5. **Stay out of the lidar's view** (below about car-roof height) while each spot settles.
6. Measure only at spots *after* the route's first node: before node 0, the nearest path point is the end of the loop.

Commands. On the PC:
```
scp qcar/check_pose.py nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/
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
- **Localization depends on the room.** Moving walls or landmarks breaks it (§2). The correction table is only valid for the current calibration.
- **The correction table only fixes the average offset.** The ±2–3 cm wander on the lower part remains. Camera lane correction (§7) is the way past this.
- **Along-lane offset.** The GPS reads about 0.2 m further along the lane than the mat markers. This doesn't affect lane keeping, but matters for start placement (§3).
- **Stray readings.** Single wrong GPS readings appear, mostly while the car is moved by hand. The GPS-jump check ignores them while driving. A loss lasting more than 2 s stops the car rather than letting it steer off.
- **Startup pause.** The car rolls a few cm, pauses ~2 s, then continues, at the same spot every run. Most likely YOLO detecting something near the start (`vGain` drops). Harmless.
- **Argument types.** `-c` and `-n` in `vehicle_control.py` are read as strings: `-n 0` selects the *second* node set and `-c False` counts as true. Use `-c 1` to calibrate and omit `-c` otherwise. `-n` has no effect, since `nodeSequence` is hard-coded.
- **Debug output.** The `GPS:` print fills the log; it can be removed or rate-limited once tuning is finished.

---

## 7. Next steps

1. **Camera lane correction:** use camera lane detection to correct the remaining few cm of sideways error against the painted lines, with the map-based controller still handling the route. This is the way past the GPS accuracy limit, and the long-term replacement for the correction table. Plan: [plans/camera_lane_correction.md](plans/camera_lane_correction.md) (builds on skill activity 06, [skills_activities/06-lane_keeping/](../skills_activities/06-lane_keeping/)).
2. **Fine-tune `gps_corrections`** in 1–2 cm steps if needed (§4).
3. **Per-car settings.** Move `steering_trim`, `gps_scale` and `gps_corrections` out of the code:
   - `steering_trim` and `gps_scale` as per-car lists in `config.txt`, in the same order as `QCAR_IPS`, passed through `run_all_cars.bat` → `start.py` → `vehicle_control.py` arguments.
   - `gps_corrections` is a table, so a per-car file is simpler than a `config.txt` list.
   - Add `type=float`/`type=int` on all arguments. This also fixes the `-c`/`-n` issue (§6).

   Required before adding car B.
4. **Calibrating a new car.** Repeat the car A procedure:
   1. Calibrate.
   2. Start ~50 cm before the first node's marker.
   3. Run with `steering_trim = 0` and read the average `cross_track`.
   4. Set `steering_trim ≈ average × K_stanley / v_ref` (a negative average gives a negative trim; car A: −0.037 × 1.5 / 0.4 ≈ −0.14).
   5. Survey with `check_pose.py`, check `gps_scale`, and build its own correction table (§2).
5. **Optional: walls near the lower part** of the map, to reduce the wander. This needs a recalibration, a full re-survey and a new correction table (§2).
6. **Multi-vehicle.** Once the above are in place: separate routes per car, then coordination at shared intersections.

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
