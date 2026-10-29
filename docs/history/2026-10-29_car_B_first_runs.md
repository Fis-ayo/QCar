# Car B: First Runs with Car A's Code

> **Historical record (2026-10-29).** How car A's working code was brought up on car B. Kept as written. Current state: [car_B_setup.md](../car_B_setup.md).

Car B (`192.168.0.152`) was given car A's [qcar/vehicle_control.py](../../qcar/vehicle_control.py) without changes: route `[0, 2, 4, 14, 16, 18, 12, 0]`, `v_ref = 0.4`, and car A's `steering_trim`, `gps_scale` and `gps_corrections`.

**Outcome:** after a library fix and a recalibration, car B completed a full lap of the route and stayed in its lane, riding closer to the outer edge than expected. Good enough for now.

---

## 1. Run 1: crash at start-up

```
File "vehicle_control.py", line 293, in controlLoop
    startNodeReached, initWaypointSequence = roadmap.initial_check(initPose, nodeSequence,waypointSequence)
AttributeError: 'SDCSRoadMap' object has no attribute 'initial_check'
```

Car B's Quanser libraries were older than car A's, and their `SDCSRoadMap` had no `initial_check`.

**Fix:** car A's library contents (`hal`, `pal`) were copied into car B's library folder, which kept **car B's folder name** (car B's Python search path points at it).

## 2. Run 2: localization lost on 4 → 14

Start at GPS (0.01, 0.50, −91°).

- **0 → 2 → 4 and the start of 4 → 14 were fine:** `cross_track` up to 6 cm, mostly 1–4 cm.
- **y ≈ 2.0–2.4:** six GPS readings came in about 0.8 m ahead of the estimate and were rejected. Then GPS came back to sensible values.
- **From y ≈ 3.1:** GPS put the car at (0.5, 4.6, 178°) and then (−0.9, 2.1, −95°), other parts of the map facing other ways. All were rejected, and after 2 s the GPS-jump check stopped the car at estimate (2.28, 3.93).

**Reading:** the lidar scan no longer matched the reference recorded at calibration. Car B had not been calibrated for the current map layout. The GPS-jump check worked as designed.

## 3. Run 3: full lap

**Car B was recalibrated before this run** (at (0, 2) facing −90°, against the current map layout).

Log: [logs/car B/vehicle_control_2026-10-29_full_lap.log](../../logs/car%20B/vehicle_control_2026-10-29_full_lap.log). Start at GPS (−0.01, 1.96, −90°). The car drove straight down to node 0 and the first `pose=` line appeared at y ≈ 0.35.

- **Lap 1 completed** 0 → 2 → 4 → 14 → 16 → 18 → 12 → 0 with 0 GPS readings rejected. `cross_track` was −2.4 … +6.1 cm, average +2.2 cm, positive on 66 of 72 lines.
- **Lap 2** (the controller is cyclic) followed the same way, average +2.0 cm, until it was **stopped by hand** on the 4 → 14 lane at estimate (2.28, 3.88). Stopping shuts down the GPS on the car, so the log ends with no further readings and the 2 s timeout's `STOPPED:` line.
- **On the mat:** closer to the outer edge than expected, but in its lane and following the route.

**Outer-edge offset:** the steady positive `cross_track` (right of path) fits a constant steering bias. Car B uses car A's `steering_trim = -0.14`, which steers right to cancel car A's leftward bias. Car A's `gps_corrections` were also measured for car A's calibration only. Neither has been measured for car B yet.

**4 → 14, y ≈ 3–3.9:** after the recalibration, car B drove this stretch with 0 rejected readings, where run 2 (before recalibrating) had lost its position.

No code changes were made for car B.

## 4. Next steps

Moved to [car_B_setup.md §6](../car_B_setup.md#6-next-steps): measure car B's `steering_trim`, survey car B with `check_pose.py`, then per-car settings.
