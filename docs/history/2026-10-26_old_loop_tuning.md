# Car A: Single-Vehicle Route Following

> **Historical record (2026-10-26).** Car A tuned on the old loop `0 → 2 → 4 → 6 → 0`. Kept as written; settings here (e.g. `gps_scale = 1.035`) are **not** current. Current state: [car_A_setup.md](../car_A_setup.md). "Setup doc §…" references in later records point to the sections of this file.

How we got one physical QCar 2 (car A, `192.168.0.151`) to drive a fixed route and stay in its lane, what was wrong along the way, and the settings that came out of it.

**Result:** car A drives the loop `0 → 2 → 4 → 6 → 0` continuously and stays in its own lane. The body stays inside the lane; on the left curves the inside rear wheel can still touch the lane divider. That is close to the accuracy limit of GPS-based lane keeping on this mat (see [Known limitations](#6-known-limitations)).

**Verified at `v_ref = 0.2` and `0.4` m/s.**

---

## 1. Current configuration

All values are in [qcar/vehicle_control.py](../../qcar/vehicle_control.py).

| Setting | Value | What it does |
|---|---|---|
| `nodeSequence` | `[0, 2, 4, 6, 0]` | Fixed route (replaces the original random node selection, which is commented out) |
| `v_ref` | `0.4` m/s | Target speed (was `0.75`, too fast for the curves). Final setup verified at `0.2` and `0.4` |
| `K_stanley` | `1.5` | Stanley steering gain (was `0.7`) |
| `steering_trim` | `-0.14` rad | Cancels car A's leftward steering-servo bias; added only in `qcar.write(u, delta + steering_trim)` |
| `gps_scale` | `1.035` | GPS x/y are divided by this before use, to line the localization up with the roadmap |
| Tracked point | `0.1` m ahead of the pose | The point the Stanley controller keeps on the path (`p = ... * 0.1`, was `0.2`) |
| `calibrationPose` | `[0, 2, -π/2]` | Where the car must sit when calibrating |

`steering_trim` and `gps_scale` were measured on **car A only**. Other cars need their own values (see [Next steps](#7-next-steps)).

### Files changed

| File | Change |
|---|---|
| [qcar/vehicle_control.py](../../qcar/vehicle_control.py) | Settings above; `gps_scale` applied to GPS x/y in two places (start-position check and EKF update); `steering_trim` in `qcar.write`; two debug prints (§3.1); `self.ect` stored in `SteeringController.update` for the print |
| [python/start.py](../../python/start.py) | `vehicle_control.py` output saved to `vehicle_control.log` on the car (§3.1) |
| [tools/on_car/check_pose.py](../../tools/on_car/check_pose.py) | New: static localization check (§5) |

---

## 2. How to run car A

1. **Calibrate** (after the mat or lidar setup changes, or if the car drives noticeably off):
   - Place car A at **(0, 2) facing −90°** (pointing down the map, just above node 11). Mark this spot and direction with tape and line the car up exactly; small placement errors here grow across the map.
   - On the car over SSH: `python vehicle_control.py -c 1`. The script exits once calibration is done.
   - Calibration is stored **on that car**. Calibrating one car does nothing for another.
2. **Place the car at the start:** in node 0's lane, about **30 cm before node 0** (around y ≈ 0.45), facing down the lane (−90°). Don't place it on or past node 0 (see [§3.2](#32-the-car-drove-a-full-extra-lap-before-starting-the-route)).
3. **Start:** **save all files first**, then run `run_all_cars.bat` on the PC. It reads [config.txt](../../config.txt) and calls [python/start.py](../../python/start.py), which uploads every `qcar/*.py` file to the car at launch. A change saved after launching is not used in that run. (This happened once: the first `steering_trim` test ran without the trim.)
4. **Watch the log live** (optional), from a second terminal on the PC:
   ```
   ssh nvidia@192.168.0.151
   tail -f /home/nvidia/Documents/self_driving/multi_vehicle_self_driving/vehicle_control.log
   ```
5. **Stop:** `stop_all_cars.bat`.
6. **Copy the log to the PC** (each run overwrites it):
   ```
   scp nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/vehicle_control.log .
   ```

---

## 3. What we found and changed

Each problem is listed with the evidence that identified it and the fix.

### 3.1 No visibility into what the car was doing
- **Problem:** `start.py` launches `vehicle_control.py` in the background over SSH and closes the connection, so all output, including crash messages, was discarded.
- **Fix:** [python/start.py](../../python/start.py) now runs `python -u vehicle_control.py > vehicle_control.log 2>&1 &`. `-u` stops Python buffering output so lines appear immediately.
- `vehicle_control.py` prints two debug lines:
  - `GPS: (x, y, heading)`: every GPS reading, already divided by `gps_scale` (logs from before that change show unscaled values).
  - `pose=… path_pt=… cross_track=…`: twice per second while on the route. `cross_track` is the sideways distance (m) between the tracked point and the path. **Negative = car left of the path.**

> **Important:** `cross_track` is measured against the car's *own estimate* of where it is. It cannot detect localization errors. A log can show 1 cm tracking while the real car is in the wrong lane. Always compare the log with what the car is physically doing.

### 3.2 The car drove a full extra lap before starting the route
- **Evidence:** when placed on node 0, the first GPS reading was ~20 cm *past* node 0. The `pose=` lines (main route) only appeared after a full lap of `GPS:` lines.
- **Cause:** `roadmap.initial_check` decides the car hasn't reached the start node yet and drives an approach path to it, which from just past node 0 means going all the way around. Early observations of "cutting into the other lane" were made during this approach lap, not on the route.
- **Fix:** start the car about 30 cm *before* node 0. The route then starts immediately.

### 3.3 Constant pull to the left: steering servo bias
- **Note:** the first gain tests (`K_stanley` 0.7, 1.0, 1.5, with no visible difference) were made before §3.2 was found, so they mostly showed the approach lap. The comparison below is from clean runs at `v_ref = 0.4`.
- **Evidence:** `cross_track` was always negative (car left of path), on straights and curves. Doubling the gain halved it:

  | `K_stanley` | Average `cross_track` |
  |---|---|
  | 0.7 | ≈ −0.08 m |
  | 1.5 | ≈ −0.037 m |

  A Stanley controller with a constant angle bias settles at `offset ≈ v · tan(bias) / K`. Both runs give the same bias, **≈ 0.14 rad (8°)**. With `delta = 0` commanded, the front wheels visibly pointed slightly left.
- **Fix:** `steering_trim = -0.14`, applied only in the `qcar.write` call so the EKF still receives the steering angle the wheels actually reach.
- **Result:** average `cross_track` ≈ +0.002 m, typical error ≈ 1.6 cm, signs evenly mixed.

### 3.4 Tracking looked perfect, but the car still cut into the other lane: localization scale
- **Evidence:** with ~1.5 cm tracking in the log, the car still crossed the divider, worst on the 2 → 4 curve. Halving the speed (`v_ref = 0.2`) changed nothing, which ruled out localization lag.
- **Static check:** [tools/on_car/check_pose.py](../../tools/on_car/check_pose.py) (see [§5](#5-diagnostic-tools)), with the car held still and centered in its lane at several spots, before and after recalibrating:

  | Spot | Round 1 | Round 2 (recalibrated) |
  |---|---|---|
  | Mid 0 → 2 curve | ≈ 0.00 m | (localization lost) |
  | Near node 2 | −0.027 m | −0.021 m |
  | Mid 2 → 4 curve | −0.069 / −0.098 m | −0.078 m |
  | Near node 4 | (localization lost) | −0.080 m |

  Jitter at rest was about ±2–3 cm. The offset **did not change with recalibration** and **grows with distance from the map origin**: GPS read x = 2.34 for a car centered in the x = 2.25 lane (≈ 4%), and y = −1.11 for the y = −1.08 lane (≈ 3%). The controller drives the *estimate* onto the path, so a GPS reading 8 cm too far out puts the real car 8 cm toward the divider.
- **Fix:** `gps_scale = 1.035`. GPS x and y are divided by it, both for the start-position check and for the EKF.
- **Result:** the car stays mostly in its lane; parts of it still crossed the divider on curves.

### 3.5 Body cutting the inside of curves: tracked point
- **Cause:** the controller keeps a point 0.2 m ahead of the pose on the path. On a curve the rest of the car follows a tighter arc, so the body cuts toward the inside, which on this left-turn loop is toward the oncoming lane.
- **Fix:** tracked point moved to `0.1` m.
- **Result:** only the inside rear wheel touches the divider on curves. Average `cross_track` moved slightly to the outer side (+0.009 m), typical error ≈ 1.5 cm.

### Summary of runs

Each row adds to the one above. All runs used the route `0 → 2 → 4 → 6 → 0`, started ~30 cm before node 0.

| Change | `v_ref` | Average `cross_track` | What the car did |
|---|---|---|---|
| Starting point (`K = 0.7`, no fixes) | 0.4 | ≈ −0.08 m | Over the divider |
| `K_stanley = 1.5` | 0.4 | ≈ −0.037 m | Better, still left-biased |
| + `steering_trim = -0.14` | 0.4 | ≈ +0.002 m | Log good; car still cut into the other lane on curves |
| Same, slower (diagnostic) | 0.2 | ≈ +0.003 m | Same cutting; fully in the other lane on the 2 → 4 curve. Ruled out lag |
| + `gps_scale = 1.035` | 0.2 | ≈ 0.00 m | Mostly in lane; parts over the divider on curves |
| + tracked point `0.1` | 0.2 | ≈ +0.009 m | In lane; inside rear wheel touches divider on curves |
| Same, final speed | 0.4 | (log not reviewed) | Works the same as at 0.2 |

---

## 4. Tuning options and trade-offs

All of these work. Which to use depends on which edge of the lane you'd rather get close to.

### `gps_scale`
Larger values pull the estimate toward the map origin, so the real car moves *away* from the origin: toward the outer edge of the mat and away from the divider on this route. Each +0.01 shifts the car about 1 cm on the bottom straight and about 2 cm on the right side; there is almost no effect near x = 0.

| Value | Trade-off |
|---|---|
| **1.035** (current) | Balanced; inside rear wheel can touch the divider on curves |
| 1.04 | ~0.5–1 cm further from the divider |
| 1.045 | ~1–2 cm further from the divider; watch the outer edge on the right side |

### Tracked point (`* 0.2` / `* 0.1` on the `p = …` line)
| Value | Trade-off |
|---|---|
| 0.2 (original) | Front of the car follows the path closely; body and rear cut the inside of curves |
| **0.1** (current) | Body stays in lane; front swings slightly wider; only the rear wheel touches the divider |

### `K_stanley`
Higher values correct sideways error harder. Too high causes zig-zagging on straights. 1.5 showed no wobble at 0.2–0.4 m/s. The correction gets stronger as speed drops (the term is `K · error / speed`), so retune if `v_ref` changes a lot.

### `v_ref`
With `steering_trim` in place, 0.2 and 0.4 m/s gave the same tracking (typical error 1.4 vs 1.6 cm). The final setup (with `gps_scale` and tracked point `0.1`) also works at both 0.2 and 0.4. 0.75 was too fast for the curves with the original settings and hasn't been retested since the fixes.

---

## 5. Diagnostic tools

### `vehicle_control.log`
See [§3.1](#31-no-visibility-into-what-the-car-was-doing). Useful patterns:
- Long run of `GPS:` lines before any `pose=` line: the car is driving the approach path, not the route (§3.2).
- `cross_track` consistently one sign: a constant bias (steering trim or heading).
- `cross_track` small while the car is visibly off: localization or path problem; use `check_pose.py`.

### `check_pose.py`
[tools/on_car/check_pose.py](../../tools/on_car/check_pose.py) reads only the GPS (motors are not used) and prints, averaged over the last 25 readings (~5 s):
- `side_offset`: sideways distance from the route path. **Positive = left of the path.**
- `spread`: min..max over the window, i.e. jitter at rest.
- `heading_err`: car heading minus path direction.
- `n`: window fill. It resets automatically when the car is moved more than 10 cm; read values only at `n=25/25`.

Run it on the car (not through `run_all_cars.bat`, which would also start driving):
```
scp tools/on_car/check_pose.py nvidia@192.168.0.151:/home/nvidia/Documents/self_driving/multi_vehicle_self_driving/
ssh nvidia@192.168.0.151
cd /home/nvidia/Documents/self_driving/multi_vehicle_self_driving
python -u check_pose.py | tee check_pose.log
```
Place the car centered in its lane, pointing along it, wait for `n=25/25`, then **roll** (don't lift) it to the next spot. Keep away from the lidar while it settles. Measure only at spots *after* the route's first node: before node 0, the nearest path point is the end of the loop, so the readings there are misleading. `nodeSequence` and `calibrationPose` at the top must match `vehicle_control.py`. `gps_scale` is **not** applied in this script, so it shows the raw localization offset.

---

## 6. Known limitations

- **Tight lane clearance.** Lanes are ~27 cm wide and the car is ~19 cm wide, leaving ~4 cm per side. Tracking error (~1.5 cm) plus GPS jitter (~2–3 cm) use most of it, so occasional contact with a lane line is expected with GPS-only lane keeping.
- **Localization dropouts.** During `check_pose.py` runs, the GPS twice jumped to a far corner of the map. Once it recovered after the car was moved; once it never recovered for the rest of the session. Both happened while the car was being handled, and none has been seen while driving, but there is currently **no safety check**: if it happens while driving, the car will steer toward the wrong place.
- **Startup pause.** The car rolls a few cm, pauses ~2 s, then continues, at the same spot every run. Most likely YOLO detecting something near the start (`vGain` drops). Harmless.
- **Argument types.** `-c` and `-n` in `vehicle_control.py` are read as strings: `-n 0` selects the *second* node set and `-c False` counts as true. Use `-c 1` to calibrate and omit `-c` otherwise. `-n` currently has no effect, since `nodeSequence` is hard-coded.
- **Debug output.** The `GPS:` print fills the log; it can be removed or rate-limited once tuning is finished.

---

## 7. Next steps

1. **Per-car settings.** Move `steering_trim` and `gps_scale` out of the code into per-car lists in `config.txt` (same order as `QCAR_IPS`), passed through `run_all_cars.bat` → `start.py` → `vehicle_control.py` arguments, with `type=float`/`type=int` on all arguments (also fixes the `-c`/`-n` issue). Required before adding car B.
2. **Calibrating a new car.** Repeat the car A procedure: calibrate, start ~30 cm before the first node, run with `steering_trim = 0`, read the average `cross_track`, and set `steering_trim ≈ average × K_stanley / v_ref` (a negative average gives a negative trim; car A: −0.037 × 1.5 / 0.4 ≈ −0.14). Then check `gps_scale` with `check_pose.py`.
3. **GPS-jump safety stop.** Stop the car if a GPS reading jumps further than the car could physically have moved since the last one.
4. **Camera lane correction** (skill activity 06, [skills_activities/06-lane_keeping/](../../skills_activities/06-lane_keeping/)). Use camera lane detection to correct the remaining few cm of sideways error against the painted lines, with the map-based controller still handling the route. This is the way past the GPS accuracy limit.
5. **Multi-vehicle.** Once the above are in place: separate routes per car, then coordination at shared intersections.
