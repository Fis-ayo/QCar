# Car B: Current Setup

How car B (`192.168.0.152`) is set up **now**: settings, how to run it, how to check it, and what's next. This is the doc to keep up to date. How each setting was found is in the [history records](#7-history).

**Status (2026-10-29):** car B drives the route `0 → 2 → 4 → 14 → 16 → 18 → 12 → 0` with **car A's code and settings, unchanged**, at `v_ref = 0.4`. One full lap was completed: 0 GPS readings rejected, `cross_track` within −2 … +6 cm, average +2.1 cm. **The car rides closer to the outer edge than expected** (seen on the mat, and `cross_track` is positive 92 % of the time), but it stays in its lane and follows the route. This is acceptable for now. The run was stopped by hand during the second lap (§4).

---

## 1. Current configuration

Car B runs the same [qcar/vehicle_control.py](../qcar/vehicle_control.py) as car A, so all settings are **car A's values**. See [car_A_setup.md §1](car_A_setup.md#1-current-configuration) for what each one does.

| Setting | Value on car B | Measured for car B? |
|---|---|---|
| `nodeSequence` | `[0, 2, 4, 14, 16, 18, 12, 0]` | n/a (route) |
| `v_ref` | `0.4` m/s | n/a |
| `K_stanley` | `1.5` | Car A's value, works on car B |
| `steering_trim` | `-0.14` rad | **No.** Car A's servo bias. Likely part of car B's outer-edge offset (§5) |
| Tracked point | `0.1` m ahead | Car A's value |
| `gps_scale` | `1.0` | **No.** Car A's calibration |
| `gps_corrections` | car A's 6 rows | **No.** Measured for car A's calibration on the lower part of the map. Kept on purpose until camera lane correction replaces it (§6) |
| `gps_max_jump` / `gps_max_heading_jump` / `gps_timeout` | `0.3` m / 30° / `2.0` s | Shared safety settings |
| `calibrationPose` | `[0, 2, -π/2]` | Same spot as car A |

[config.txt](../config.txt) currently points only at car B: `QCAR_IPS=[192.168.0.152]`, `CALIBRATE_IP` and `PROBING_IP` = `192.168.0.152`, `LOCAL_IP=192.168.0.153`.

### Car B software difference: Quanser libraries

Car B's original Quanser Python libraries were an older version. Their `SDCSRoadMap` (`hal.products.mats`) had no `initial_check`, so `vehicle_control.py` crashed at start-up (`AttributeError: 'SDCSRoadMap' object has no attribute 'initial_check'`).

**Fix:** car A's library contents (`hal`, `pal`) were copied into car B's library folder. The folder kept **car B's own name**, because car B's Python search path points at that name.

Check on the car (should print `True`):
```
python3 -c "from hal.products.mats import SDCSRoadMap; print(hasattr(SDCSRoadMap, 'initial_check'))"
```

---

## 2. Room layout requirements

Same room and mat as car A: see [car_A_setup.md §2](car_A_setup.md#2-room-layout-requirements). Calibration is stored on each car, so **car B needs its own calibration** after any change to the room, the mat or its lidar.

**Car B's current calibration:** done 2026-10-29, just before the full-lap run (§4), with the current map layout.

---

## 3. How to run car B

Same procedure as [car A §3](car_A_setup.md#3-how-to-run-car-a), with car B's IP:

1. **Calibrate** if needed: car B at **(0, 2) facing −90°**, then on the car `python vehicle_control.py -c 1`.
2. **Place the car at the start**, about 50 cm before the node 0 marker, facing down the lane. (The 2026-10-29 run started further back, at GPS y ≈ 1.96. That works: the car drives straight down to node 0 first. The first `pose=` line appears at y ≈ 0.35.)
3. **Start:** save all files, then run `run_all_cars.bat`.
4. **Watch the log** (optional):
   ```
   ssh nvidia@192.168.0.152
   tail -f /home/nvidia/Documents/self_driving/multi_vehicle_self_driving/vehicle_control.log
   ```
5. **Stop:** `stop_all_cars.bat`, or the car stops itself after 2 s without an accepted GPS reading.
6. **Copy the log** with its own name:
   ```
   scp nvidia@192.168.0.152:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/vehicle_control.log "logs/car B/vehicle_control_<name>.log"
   ```

---

## 4. Latest run (2026-10-29)

Car B was **recalibrated just before this run**, against the current map layout. The run before it, without recalibration, lost localization on 4 → 14 ([history §2](history/2026-10-29_car_B_first_runs.md#2-run-2-localization-lost-on-4--14)).

Log: [logs/car B/vehicle_control_2026-10-29_full_lap.log](../logs/car%20B/vehicle_control_2026-10-29_full_lap.log)

| | Lap 1 (full route) | Lap 2 (partial) |
|---|---|---|
| `cross_track` range | −2.4 … +6.1 cm | −1.9 … +4.3 cm |
| Average `cross_track` | +2.2 cm | +2.0 cm |
| Positive (right of path) | 66 of 72 lines | 36 of 39 lines |
| GPS rejected | 0 | 0 |

- **Lap 1** went 0 → 2 → 4 → 14 → 16 → 18 → 12 → 0 without problems. The controller is cyclic, so the car then started the route again.
- **Lap 2** was **stopped by hand** on the 4 → 14 lane, at estimate (2.28, 3.88). After the last `GPS:` line (y ≈ 3.09) no readings arrived and the log ends with `STOPPED: no GPS reading accepted for 2 s`. That is what a manual stop looks like in the log: `stop_all_cars.bat` shuts down the GPS on the car, and the 2 s timeout prints the `STOPPED:` line. It is not a localization problem.
- **On the mat:** the car was closer to the outer edge than expected but stayed in its lane and followed the route.

---

## 5. Known limitations

- **Outer-edge offset.** `cross_track` is positive (right of path, toward the outer edge) almost all the time, around +2 cm. The likely cause is that car B uses car A's `steering_trim = -0.14`, which cancels a *leftward* bias car A has. If car B doesn't have that bias, the trim pushes it right. Car A's `gps_corrections` may also not fit car B's calibration. `cross_track` is measured against the car's own estimate, so the real offset on the mat can be bigger than the log shows (see [car A §5](car_A_setup.md#5-diagnostic-tools)).
- **Calibration and the 4 → 14 lane.** Before recalibrating, car B lost its position on the upper part of 4 → 14 (y ≈ 3–3.9): GPS readings jumped to other places on the map. After recalibrating, it drove this lane without a single rejected reading. This is the same lane where car A's localization depended on the walls and landmark near node 14, so recalibrate car B after any change around the mat.
- **A manual stop ends the log with `STOPPED:`.** Stopping the cars shuts down the GPS, so the 2 s timeout prints `STOPPED: no GPS reading accepted for 2 s` even though nothing went wrong. A run stopped this way shows no `GPS rejected:` lines before the stop.
- **YOLO start-up message.** `YOLO Server error: status check failed` appears twice at start, followed by `Reconnected to yolo Server`. Harmless.
- All of [car A's limitations](car_A_setup.md#6-known-limitations) also apply (tight lane clearance, room-dependent localization, `-c`/`-n` argument types).

---

## 6. Next steps

1. **Car B `steering_trim`** (when the outer-edge offset needs fixing): follow [car A next step 4](car_A_setup.md#7-next-steps). A first estimate from this run: `steering_trim ≈ -0.14 + 0.021 × 1.5 / 0.4 ≈ -0.06`. Check it with a drive at `v_ref = 0.2` first, and compare with what the car does on the mat.
2. **`gps_corrections`: keep car A's table on car B for now.** Decided 2026-10-29: camera lane correction ([plans/camera_lane_correction.md](plans/camera_lane_correction.md)) is the permanent fix and will replace the table, so no car B table will be built. Not started yet.
3. **Per-car settings** ([car A next step 3](car_A_setup.md#7-next-steps)): needed before car A and car B can run together, at least for `steering_trim` once car B has its own value.
4. **Shared doc:** move what both cars share (room layout, survey procedure, diagnostics) out of `car_A_setup.md` into a shared doc, as planned in the [docs README](README.md).

---

## 7. History

| Record | What it covers |
|---|---|
| [history/2026-10-29_car_B_first_runs.md](history/2026-10-29_car_B_first_runs.md) | Getting car A's code to run on car B: library fix, localization loss on 4 → 14, first full lap |
