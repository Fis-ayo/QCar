# Camera Lane Correction: Report

> **Combined report for the [camera lane correction plan](../plans/camera_lane_correction.md)**, car A (`192.168.0.151`), from 2026-11-10. New work on this plan is added here as a new dated part, so all results stay in one doc. Current settings: [car_A_setup.md](../car_A_setup.md).

**Goal:** see the left and right lane boundaries with a camera, so the car can stay centred in its lane without depending on the lidar GPS and its correction table. Since 2026-11-11 the goal is for the camera to **replace** the `gps_corrections` table completely, not blend with it.

**Status (end of 2026-11-19):**
- **Camera steering** (step C3, `use_camera_steer = True`, §35–§40): the car steers on the front camera's offset and heading to the lane; the GPS gives only the position along the route (nudges when it agrees), no correction table.
- **2026-11-19, general code instead of place-specific fixes:** no planned stop (§42), no start zone before node 0 (the car can start anywhere on the route, pointing along it), no `cs_edge_ignore`, no GPS correction table (§43). Curvature feedforward tried and removed (§44–§46).
- **Drive of 2026-11-19 (§46):** ~10.5 m at 0.4 m/s to node 16, the furthest so far. The car left its lane in the roundabout: the front camera was blind there for 2.3 m, and (§47) with the old trim it could only reach ~21° of left steering where the roundabout needs 22.3°, so most likely it went wide (outward). Along-route position ~15–17 cm ahead by node 16 after the GPS loss on 4 → 14. ~5 cm dead-reckoning drift through 0 → 2 (first fit at node 2: −8 cm).
- **Since then (§47, not yet driven):** `steering_trim` −0.14 → **−0.04**, `K_soft` 0.05 in Stanley, curvature speed limit written but **off** (`use_curve_speed = False`). Each has a one-line revert (§47 table).

**Where to pick up: two test drives, one change at a time (agreed with the user on 2026-11-19), then the left camera.**

1. **Drive A: new trim only** (`steering_trim = -0.04`, `K_soft = 0.05`, `use_curve_speed = False`, `v_ref` 0.4). Same start as §46 (straight before node 0, procedure car_A_setup.md §3). Copy back the whole log set (`vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv`, `lane_debug/`, `raw_frames/`), and note **where the car leaves the lane, which way (inward/outward), and where it stops**. Check in the logs:
   - straights (approach, 4 → 14): commanded `delta` ≈ 0 (was +6°) and `cs_d_cm` ≈ 0 (was −1 … −2 cm). If the car now pulls left, the trim went too far: back to −0.14 and look at the log;
   - achieved steering atan(L · gyro / speed) vs needed in the roundabout (22.3°) and on 16 → 18 (24.4° right), and no long stretches at ±30°;
   - node 2: is the first fit after 0 → 2 still ~−8 cm?
2. **Drive B: switch on the speed limit** (`use_curve_speed = True`; straights 0.40, 0 → 2 0.34, roundabout 0.30, 16 → 18 0.28 m/s) and compare the same curves with drive A. The simulation predicts smaller offsets only if the camera/servo delays are large (§47); this drive tells whether it is worth keeping.
3. **Confidence-weighted sensor fusion: GPS + camera + encoder/gyro + map (agreed with the user on 2026-11-19, before the left camera).** The user's direction: **the GPS stays an integral part of the project** (real self-driving cars use GNSS too, and a camera alone has its limits); no single sensor should be relied on alone. Now a camera fit either replaces the offset completely (`cs['d'] = mf_d`) or is ignored, and dead reckoning carries on until a fixed `cs_dr_max` (2.3 m). Instead, a small Kalman filter on offset and heading:
   - prediction by encoder + gyro, with an uncertainty that grows with distance, faster on tight curves (an along-route error Δs gives a heading error κ·Δs, §46);
   - each camera fit pulls the estimate in proportion to its confidence (fit rms, `both`/`YE` vs edge-only, curvature change in view), replacing the jump gate, confirmation and re-lock rules with one test on the innovation;
   - the safety stop triggered by the uncertainty (e.g. > ~3 cm offset sd), not by a fixed distance: earlier on tight curves, later on straights;
   - **GPS as a full input, along and across the route**, with an honest uncertainty instead of a per-place table: it holds the position along the route (§46: ±3 cm while good, ~3 % drift without it), gives the start position, cross-checks the camera, and carries the car where the camera is blind. Readings that disagree with the estimate beyond their uncertainty are rejected (the false fixes on 4 → 14 would fail this test), and the uncertainty can be learned per drive from how the GPS agrees with the camera on straights;
   - in the same filter, the position along the route (s) as a state, corrected by the GPS and later by camera curve starts/ends.
   Test offline first on the §40 and §46 logs with `c3_replay.py`. It also prepares for two cameras with different confidence.
   - **Related GPS work:** the false fix on 4 → 14 (x ≈ 3.2 instead of 2.28 in every drive since 2026-11-11, §6, §46) was deferred (§13). With the GPS as an integral sensor it should be looked at: `check_pose.py` survey of 4 → 14, the room near x ≈ 3.2, y ≈ 2, and whether a landmark or recalibration removes it.
4. **Then the left camera for the curves (plan step 3b)**, below: both blind stretches (0 → 2, the roundabout) are left curves, and the §46 left frames show the roundabout lane clearly. Still open after that: the along-route position drift without GPS (§46), and the rear axle running 3.8 cm inside on 0 → 2 (§44).

**Left camera (plan step 3b), chosen by the user on 2026-11-18.** Goal: measure the lane beside the car in curves like 0 → 2 and the roundabout, where the front camera is blind, so they're driven on a measurement instead of dead reckoning.

What's already known:
- **What it sees** (§1, §14, §18): the yellow line as an arc in the **bottom corners** of the image. The part directly beside the car is below the frame. The bottom-left part is the line just ahead-left of the front axle, where the front camera is blind. Useless when someone walks beside the car (§17).
- **Recordings with left frames** (`_left.jpg`, 5/s, same `t_epoch` names as the front frames): every `raw_frames/2026111*` folder, and `logs/car A/Fresh_Log/20261119_234304/` (the §46 drive, through the roundabout to node 16, with its own logs). **`20261118_234715` is the best one for 0 → 2:** a camera-steering drive with its `odometry_drive.csv` (encoder, gyro, `cs_s`, `cs_d_cm`, `cs_h_deg`), `lane_camera.csv` and `vehicle_control.log` all still in `logs/car A/`. `20261117_204844` is the hand-rolled centred pass (true offset ≈ 0) with `odometry.csv`.
- **Calibration:** the left camera has none yet. The front camera needed a checkerboard (§26–§28: borrowed intrinsics were badly wrong, non-square pixels FY/FX ≈ 0.665). Expect the same for the left camera.

Suggested steps:
1. **Checkerboard calibration of the left camera:** add a camera option to [qcar/calib_capture.py](../../qcar/calib_capture.py) (it only reads `csiFront` now), record on the desk as for the front camera (§27–§28), and solve with [qcar/calib_solve.py](../../qcar/calib_solve.py).
2. **Mounting** (height, angles, position on the car), without tape. Use frames where the front camera already gives the offset and heading: parked on a straight, and the straights of `20261118_234715` (front fits about −1 cm on 4 → 14). Fit the left camera's pose so that its yellow line lands where the front camera says.
3. **Offline test on `20261118_234715` and `20261117_204844`:** detect the yellow line in the left frames, project it to the floor, fit it with the map lane shape at `cs_s` (as `lane_camera.map_fit`), and compare with the front camera where both see the lane, and with the dead reckoning through 0 → 2. On the hand-rolled pass the answer should be ≈ 0.
4. **Then** add it to `lane_camera.py` (second camera; check the CPU, §4: frame rate and `proc_ms`) and to `vehicle_control.py`'s camera steering (left-camera fit used where the front camera has none, e.g. 0 → 2).

Later: (curvature feedforward tried in simulation, §44–§45: ramped over L it equals plain front-axle Stanley, no gain; removed §46; the rear axle runs 3.8 cm inside the front on 0 → 2, a possible target), and the route past node 14 (roundabout, right turn; each intersection needs the same checks as node 0, §39).

---

# Part A: 2026-11-10, steps 0–2

## 1. Step 0: which cameras can see the lines

[qcar/csi_snapshot.py](../../qcar/csi_snapshot.py) saved one frame from each of the four CSI cameras (820×410), plus a copy with the yellow HSV mask from [examples/QCar2_task_lane_following_probe.py](../../examples/QCar2_task_lane_following_probe.py). Positions: right lane and left lane, straight and curve. Images: `logs/car A/snapshots/`.

- **The yellow divider is always on the car's left** on this route, including the left lane, which is driven in the opposite direction.
- **The right boundary is not a painted line.** It's the edge between the dark road and the white surface beside it.
- **Front camera:** sees the yellow line ahead and the right road edge. It's the only camera that sees both boundaries.
- **Left camera:** sees the yellow line as an arc at the bottom corners of the image only. The part directly beside the car is below the frame.
- **Right camera:** sees no road at all. The road edge is in its blind spot.
- **Back camera:** not useful (intersection markings).
- **The example's yellow threshold** `[10,50,100]–[45,255,255]` works despite the pink colour cast. Its false hits are all above the horizon.

**Decision:** front camera only to start (one calibration, less CPU). The left camera becomes a later step (3b): a backup when the view ahead is blocked (car B in front) and for curves.

## 2. Step 1: calibration, no checkerboard or tape

- **Fisheye:** reused the CSI intrinsics and distortion at 820×410 from [image_interpretation.py:290](../../skills_activities/02-image_interpretation/image_interpretation.py#L290). The undistorted front images have straight poles, floor edges and lines.
- **Ruler measurements:** camera 11 cm above the floor (to the top of the camera; the lens is ~1 cm lower), 10 cm ahead of the front axle (direction confirmed 2026-11-11, §9; the distance is unchecked). Lane width 25.7 cm, from the centre of the yellow line to the road edge.
- **Pitch** from the vanishing point of the yellow line and the right edge: −2.66° and −2.68° in the two straight snapshots.
- **Centring works:** a car parked centred measured −0.2 cm and −0.4 cm from the lane centre, heading within 0.4°.
- **Scale is off ×1.38:** the projected lane width came out 35.5 cm instead of 25.7 cm. Most of this is probably the borrowed intrinsics; the height explains ~10 %. Centring isn't affected, because the error stretches both distances equally.
- **Fix used in the script:** with both lines visible, scale by the known lane width on every frame. With only one line, use a fixed factor 25.7 / 35.5 = 0.724 (confirmed in practice, §8).

The camera calibration belongs to the car, not the room (intrinsics, pitch −2.67°, 11 cm, 10 cm, 25.7 cm, scale 0.724, all in `lane_camera.py`). Re-check only if the camera is knocked or remounted.

## 3. Step 2: the measure-only script

[qcar/lane_camera.py](../../qcar/lane_camera.py): front camera, its own process, no motor or steering access.

- Undistort (precomputed remap), search only below the horizon.
- **Yellow:** HSV threshold AND near an edge (Sobel), projected onto the floor, robust line fit over the near part (up to ~70 cm ahead).
- **Right edge:** per image row, the first dark → bright step to the right of the yellow line.
- **Modes:** `both` (lane-width scaled, lane width must be within 20 % of the expected value), `yellow`, `edge`, `curve` (heading > 10° or curvature > 0.5 /m: offset not trusted, confidence 0), `none`.
- **Log:** `lane_camera.csv`, one row per frame: `t_epoch`, `mode`, `offset_cm` (+ = car left of lane centre), `heading_deg` (+ = car points left), `conf`, raw distances, lane width, curvature, point counts, fit scatter, processing time. Overlay images in `lane_debug/` every 2 s.
- **Offline test on the snapshots:** straights +0.2 / +0.3 cm, lane width 25.3 cm, confidence 1.0. **Curves: no fit works.** The line is visible only from ~28 cm ahead, over a short stretch; line, quadratic and circle fits all gave wrong offsets. So curves are flagged `curve`.

**[qcar/vehicle_control.py](../../qcar/vehicle_control.py) change:** each `pose=` line now ends with `t_epoch=` (same clock as the camera log), so the two logs can be matched by time. Note the opposite signs: `cross_track` negative = car left of the path; camera `offset_cm` positive = car left of centre. Documented in [car_A_setup.md](../car_A_setup.md).

## 4. Drive 1: GPS failure, camera script too heavy

Logs overwritten by laps A/B, except the findings here.

- The run stopped after ~10 s on the 0 → 2 curve: GPS readings 0.5–3 m off, then the 2 s safety stop.
- The camera script processed ~70 frames/s although the camera delivers ~30. **62 % were repeats of the previous frame**, keeping one CPU core busy.
- **Fix:** process only new frames (`readAll()` flags), at most 15/s (`--max-rate`).
- On the 0 → 2 curve, the yellow line is out of the front camera's view (left).

## 5. Laps A and B: the GPS problem is the localization

Logs: `logs/car A/vehicle_control_lapA_no_camera.log`, `vehicle_control_lapB_camera.log` (lap B's `lane_camera.csv` and `lane_debug/` were later overwritten).

| | Lap A (no camera) | Lap B (camera) |
|---|---|---|
| Result | full lap, 37 s | stopped after 20 s at (2.25, 3.5) |
| GPS rejections | 29 | 39 |
| Where | mostly 0 → 2 → 4, jumps of 0.5–2.5 m towards +y | same places |

- **Without the camera, the GPS also fails at the same spots.** At (2.25, 3.5), where lap B stopped, lap A also had one wrong reading with the same false solution, ≈ (0.4, 4.54, 180°).
- On 2026-10-28 the route had 0 rejections. **The room has changed since then** (confirmed by the user), so car A needed recalibrating.
- **CPU fix works:** 13.8 frames/s, median 10 ms per frame.
- **Camera vs GPS on the 4 → 14 straight** (x ≈ 2.28, y 0.9–2.7, both lines, 9 matched samples): camera +0.7 cm (sd 1.3), GPS −3.2 cm (sd 1.0, sign flipped so left = +). **Camera − GPS = +3.9 cm (sd 1.5).** The camera says centred and the overlay image agrees; the GPS says ~3 cm right. These numbers predate the recalibration.
- **Front camera blind spots on this route:** the 0 → 2 curve and its intersection (`curve`/`none`); around node 4 and the start of 4 → 14 (x ≈ 2.25, y −0.1 to 0.9), where only the right edge is seen (`edge`).
- `lane_camera.py` now empties `lane_debug/` at start.

**Plan at the end of the day:** recalibrate car A, check the localization with `check_pose.py`, a lap without the camera (target 0 rejections), then camera laps again. Done on 2026-11-11 (Part B), except a usable `check_pose` survey.

---

# Part B: 2026-11-11, recalibration, step 4, step 5

## 6. After the recalibration: 4 → 14 failure

Log: `logs/car A/vehicle_control.log` (first drive of the day; overwritten later, findings here).

- **0 → 2 → 4: 0 rejections** (laps A/B had 29–39, mostly there, §5). `cross_track` within ~6 cm.
- **4 → 14:** from y ≈ 1.0–1.3 every GPS reading is rejected. They all land on the same wrong position, **x ≈ 3.21–3.25 instead of 2.28** (~0.95 m to the right), heading 82–90°, y 0.5–1.5 m too far ahead. The localization never recovers, and the car stops after 2 s at y ≈ 1.8–2.1.
- **Repeatable:** the user saw it stop at the same place on several drives. The camera drive (§8) shows the same wrong solution at the same place.
- **Warning sign before the loss:** the readings jump 10–20 cm forwards and backwards along the straight, while x stays steady.
- It's a different wrong solution from the old room's ≈ (0.4, 4.54, 180°). Likely something in the room near x ≈ 3.2, y ≈ 2 doesn't match the new map. **Not investigated further** (deferred, §13).

## 7. `check_pose.py` attempt

Log: `logs/car A/check_pose.log`. Not usable:

- Every reading was ≈ (−0.88, 1.96, −93°), near the calibration pose (0, 2, −90°), not on 4 → 14. Most likely the script was started (or the car lifted) away from node 0, so the localization locked onto a wrong solution near its starting guess. The survey procedure says to start at node 0 and roll the car between spots ([car_A_setup.md](../car_A_setup.md) §5).
- ~15 s at one spot. `n` never passed 10/25 because y kept jumping by up to 0.9 m (1.33 to 2.26) while x and heading stayed fixed.

## 8. Camera drive (measure only)

Logs: `vehicle_control.log` and `lane_camera.csv` from 17:07:14–17:08:02 (overwritten by the step 5 drive, §11; findings here).

| Stretch | Camera mode | Clean GPS? |
|---|---|---|
| Parked at the start (29 s) | `both`: offset −1.24 cm (sd 0.05), heading −0.7°, width 25.9 cm | yes |
| Node 0 start and 0 → 2 curve | `curve`/`none` throughout | yes |
| Node 4, start of 4 → 14 (y −0.2 → 0.9) | `edge` | yes |
| 4 → 14 (y 0.9 → 1.0) | `both` | only ~0.3 s before the first rejection |
| 4 → 14 after the rejections, to the stop and parked | `both` | no (dead reckoning) |

- **Camera − GPS where both are trusted: about +2 to +5 cm** (e.g. y 0.91: camera +1.4, GPS −3.4). Same direction and size as before the recalibration (+3.9, §5). Few samples, mostly `edge` mode.
- **At the start:** the GPS put the car ~8 cm left of the path, while the camera (and the ruler, §9) said centred. The car swung ~12° right, and the camera immediately flagged `curve`.
- **After the GPS was lost:** dead reckoning held `cross_track` at about −3.6 cm, but the camera saw the car **drift left from +1.1 to +7.3 cm, heading 0 → +4.2°**. The last overlay image confirmed it. A camera correction would have caught this.
- **Single-line scale (0.724):** on 4 → 14 the offset is continuous where the mode switches from `edge` to `both` (+1.1 → +1.4 → +1.0 cm), so the fixed scale agrees with the lane-width scaling.
- **The front camera gives nothing from node 0 through the 0 → 2 curve.**

## 9. Ruler check (plan step 4)

Car width 19.5 cm (outside of the wheels). Camera **ahead of the front wheels** (confirmed; the 10 cm distance is still unchecked). True offset = 12.85 − (d + 9.75) cm, + = left of centre.

| Spot | Log | True offset | Camera | Error | Camera sd |
|---|---|---|---|---|---|
| Centred, node 0, d_left = 3.6 cm | `lane_camera.csv` of §8 (parked start) | −0.5 | −1.24 | −0.7 | 0.05 |
| Left wheel 2.5 cm from the yellow centre | `lane_camera_left_d_2.5cm.csv` | +0.6 | +1.17 | +0.6 | 0.03 |
| Right wheel 2.5 cm from the road edge | `lane_camera_right_d_2.5cm.csv` | −0.6 | +0.39 | +1.0 | 0.09 |

- All frames `both`, confidence 1.0. **All errors ≤ 1 cm**, against GPS errors of 3–9 cm.
- **Not a full check:** the car only fits ±3.1 cm in the lane with the wheels inside the lines, so the offsets tested (±0.6 cm) are about the size of the errors. The ordering is also off: "centred" read further right than the right-wheel spot. Likely ruler precision (a few mm), the spots being in different places (camera lane width 25.9 cm at node 0, 25.0 cm in the two new logs), and heading (~1.2° in the right-wheel log ≈ 0.5 cm between the front and rear wheels).

**Decision (user, 2026-11-11):** accept ±1 cm and go on to step 5. **If the camera steering doesn't behave, come back and tighten the accuracy** with this test:
- At each spot, at the **front wheels**, measure both clearances: d_left (wheel to yellow centre) and d_right (wheel to road edge). **True offset = (d_right − d_left) / 2**, which doesn't depend on the lane or car width.
- Spots on one straight: centred; left wheel on the yellow centre (≈ +3 cm) and 3 cm past it (≈ +6 cm); the same on the right. The front camera looks ahead, so a wheel on the line doesn't block it.
- One CSV per spot (`--log`).
- Optionally, turn the car ±5° about its front axle centre: if `CAM_AHEAD` is right, `offset_cm` stays put and only `heading_deg` changes.

## 10. Step 5 written: camera steering

Camera steering replacing the `gps_corrections` table (details: [car_A_setup.md](../car_A_setup.md) §1, "Camera lane correction"):
- [qcar/lane_camera.py](../../qcar/lane_camera.py) sends each result over UDP to `127.0.0.1:18555` (`--udp-port`, 0 = off).
- [qcar/vehicle_control.py](../../qcar/vehicle_control.py):
  - `use_gps_table = False`.
  - `use_camera = True`: `cam_bias` = camera − GPS offset, low-pass 0.3 s, learned while the camera is trusted (`both`, conf ≥ 0.6, < 0.2 s old, within 10 cm once driving) and held otherwise. Stanley steers from the tracked point shifted sideways by `cam_bias`.
  - Also learned while parked during `startDelay`.
  - `v_ref = 0.2`.
  - The `pose=` line gains `cam=`, `bias=`, `used=`.
- **Changed from the plan's original idea:** the table is off, and the learned bias is applied directly instead of a confidence-weighted blend.
- **Start node 0** (user's choice): the right swing at the start (§8) came from the GPS error at node 0, which the camera can fix before the car moves.

## 11. First step 5 drive (camera steering, table off)

Logs: `logs/car A/vehicle_control.log`, `lane_camera.csv`, `lane_debug/` (18:03–18:04, clean folder). `v_ref = 0.2`, start at node 0. Stopped by hand on the 2 → 4 curve after the car crossed the divider.

- **The start is fixed.** Bias learned while parked: **−5.7 cm** (camera −0.1 cm, GPS +5.8 cm, 13 messages). The car went straight (GPS heading −91 … −93° for 3 s, no right swing). For its first ~1 s of motion the camera stayed in `both` mode, offset within −1.5 cm, heading within −2.6°.
- **The front camera still loses the lane after ~1 s**, now because the 0 → 2 curve starts ahead (fitted curvature > 0.5 /m → `curve`), not because of a swing. It was blind for ~9 s (y ≈ 0.25 to the 0 → 2 exit). The bias was held at −6.5 cm.
- **The held bias wasn't right for the curve.** At the 0 → 2 exit (≈ (0.95, −0.95)) the camera saw both lines again: **offset +7.4 … +8.0 cm, heading −6 … −8°**, i.e. the left wheels ~4.5 cm over the divider ([0038.77.jpg](../../logs/car%20A/lane_debug/0038.77.jpg) shows the yellow line close on the left). Just before that, the car's estimate had it 0–3 cm right of the path. The camera moved the bias −6.1 → −2.3 cm within 0.5 s, then went blind again at node 2 / the 2 → 4 curve.
- **Conclusion:** the GPS error changes by several cm along the curves (as the old table's different rows already showed), so one bias learned at node 0 can't carry the car through 0 → 2 → 4. The curves need their own camera measurement.

## 12. Curves: discussion (no code)

**User's idea:** use whichever boundary is visible. On a curve the front camera loses only one boundary (the inner one), so steer on the other until both are back. Name the boundaries "left"/"right" rather than "yellow"/"edge", so it works in any lane (in the left lane the yellow line is still the car's left boundary).

**What the logs say:**
- On the 0 → 2 curve the camera **did keep the right edge** (`n_edge` 25–65 per frame). It reported `curve`, not `edge`, because the curve check (heading > 10° or curvature > 0.5 /m) overrides any single-line result.
- **Why the check exists:** the camera sees the line only ~30–70 cm ahead, but the offset is needed at the front axle. A straight fit to a curving line, extended back to the axle, is off by about curvature × 0.3 m × 0.7 m / 2, **≈ 10 cm for a ~1 m radius curve**. This is why line, quadratic and circle fits all failed (§3).
- **The edge does carry the information:** offsets from the right-edge distance alone go ≈ 0 at the curve start → +2 … +5 cm → +8 cm at the exit, which matches the drift the camera confirmed there.

**Proposal discussed:** a single visible edge **plus the map's curvature**. The route is known, so the curvature comes from the path: right edge radius = lane-centre radius + half the lane, yellow = − half. Fit the visible edge with the curvature fixed, leaving only offset and heading. `both` mode unchanged; hold the bias only when neither edge is visible.

**Risks to check on real frames:**
1. In intersections (node 0 exit, node 2) the right-edge search can latch onto white dashed markings ([0036.75.jpg](../../logs/car%20A/lane_debug/0036.75.jpg)), or there is no edge.
2. The edge gives only one point per image row (25–65 on curves, against ~1000 yellow pixels), so it may be noisy.

**Suggested first step (not done):** a `lane_camera.py` option to save raw frames (~5/s). Then collect (1) the car rolled slowly by hand, centred, along 0 → 2 → 4 (true offset ≈ 0, a known answer) and (2) one normal drive with the table back on for safety. Develop the curve fit offline on those.

**The user is researching a better method for the curves before continuing.**

---

## 13. Where to pick up (end of Part B)

*(As of the end of 2026-11-11. Replaced by §15.)*

1. **Curves:** the user's research, then choose a method (the §12 proposal is one option; another is the left camera, step 3b). Collect raw frames for offline testing whichever method is chosen.
2. **The code on the PC is set for camera steering** (`use_camera = True`, `use_gps_table = False`, `v_ref = 0.2`). **With these settings the car crosses the divider on the 0 → 2 curve.** For a drive that isn't testing the camera, set `use_camera = False` and `use_gps_table = True`. The table is still the old-room one, so expect a few cm of error on the lower part.
3. **Deferred:** the 4 → 14 localization failure (§6); the tighter ruler test (§9) if camera steering misbehaves on straights; independence from room changes.

**Open questions:** the camera's distance ahead of the front axle (10 cm, unchecked; the rotation test in §9 would check it). How the camera − GPS difference (§5, §8) varies around the whole route once the 4 → 14 failure is fixed.

---

# Part C: 2026-11-12, curve method assessment

## 14. Proposed pipeline: assessment (no code)

The user brought a candidate pipeline to check for feasibility (not a final choice): (1) front + left + right cameras warped into one bird's-eye view; (2) 2nd-order polynomial fits in metres, with a single visible line shifted by half a lane along its normal; (3) a 2-state Kalman filter [lateral error, heading error] predicted from tachometer + gyro, updated by the camera; (4) curvature feedforward `δ_ff = arctan(L·κ)`.

| Part | Verdict | Evidence |
|---|---|---|
| 1. Multi-camera view | Left camera yes, right camera no, no full image warp | Curve snapshots (§1) |
| 2. Metric fit + normal shift | Normal shift yes, free polynomial no | §3, §12 |
| 3. Kalman filter (tach + gyro) | Already exists; drifts too much for the curves | §8, §11 |
| 4. Curvature feedforward | Yes, from the path waypoints | Steering has none today |

**1. Multiple cameras.**
- **Right camera:** `right_lane_curve_right.png` shows only the white surface; the road edge is below the frame. The front camera keeps the right edge on the 0 → 2 curve anyway (§12). Not needed.
- **Left camera:** `right_lane_curve_left.png` shows the yellow line as an arc in the bottom-left and bottom-right corners, not "directly beside the wheel". The bottom-left part is the line just ahead-left of the front axle, which is where the front camera is blind. It could check the offset near the axle instead of extending a line from 30–70 cm ahead (the cause of the ~10 cm curve error, §12). This is plan step 3b. Its mounting angle and height must be found first (possible without a checkerboard, from the straight snapshot where the offset is known).
- **No full image warp:** `lane_camera.py` already projects only the detected pixels onto the floor. Warping and stitching three full images costs CPU (a real problem before, §4) and adds timing issues, with no extra information.

**2. Fitting.**
- Shifting a single line along its normal (not sideways) is correct and should be used. Half lane = 12.85 cm (lane 25.7 cm), not ~13.5 cm.
- The free 2nd-order fit already failed on curves (§3): the line is seen only ~28–70 cm ahead, so the fitted curvature is poorly pinned down. Taking the curvature from the map path and fitting only offset and heading (§12 proposal) covers this part.

**3. Kalman filter.**
- The EKF in `vehicle_control.py` already predicts from `qcar.motorTach` and `gyroscope[2]`, in map coordinates.
- It drifts: after the GPS was lost on 4 → 14, it held `cross_track` steady while the camera saw a 6 cm drift in a few seconds (§8). The front camera is blind ~9 s on 0 → 2 (§11).
- A lane-frame filter would also need the path curvature to predict heading. Useful for bridging a few missed frames, not for replacing a measurement on curves.
- The camera runs at 15 frames/s (`--max-rate`), not 30.

**4. Feedforward.**
- `SteeringController.update` is plain Stanley (heading + cross-track), so it only reacts once an error appears on a curve.
- `arctan(L·κ)` with κ from the path waypoints (not from the camera fit, see 2) is a few lines and helps with or without the camera. The 0.256 m wheelbase should be checked on the car.

**Limit of camera-only steering:** at intersections (node 0 exit, node 2) there are no lines, or dashed markings confuse the edge search (§12 risk 1), and the camera can't choose the turn. The map and lidar GPS are still needed for route following. The camera can replace the `gps_corrections` table, not the localization.

**Combined direction:** front camera single edge + map curvature (§12), left camera near the axle (step 3b), the existing EKF for short gaps, feedforward from the path.

## 15. Where to pick up

*(As of 2026-11-12.)*

1. **Next step: record raw frames, no steering changes.** Add an option to `lane_camera.py` to save raw frames from the front and left cameras (~5/s). Roll the car slowly by hand, centred, through 0 → 2 → 4 (true offset ≈ 0). Then test offline whether (a) the front camera's single edge + map curvature and (b) the left camera near the axle are accurate enough on the curves.
2. **Separate step, either order:** curvature feedforward in `SteeringController`.
3. **Unchanged from §13:** the code on the PC is set for camera steering, which crosses the divider on 0 → 2. For other drives set `use_camera = False`, `use_gps_table = True`. Deferred: the 4 → 14 localization failure (§6), the tighter ruler test (§9).

---

# Part D: 2026-11-16, raw frame recording

## 16. `--save-raw` option (§15 step 1, code only, not yet run on the car)

[qcar/lane_camera.py](../../qcar/lane_camera.py) has a new option, `--save-raw`:
- Also turns on the left camera. Every `--raw-every` s (default 0.2 = 5/s) it saves the raw (still distorted) front and left frames as JPEG q95 to `raw_frames/<start time>/<t_epoch>_front.jpg` and `_left.jpg`. A new folder per run, so recordings are never overwritten.
- The file name is the `t_epoch` on the same clock as `lane_camera.csv` and the `pose=` lines, so frames can be matched with the GPS pose.
- Front processing, the CSV and UDP are unchanged. The cameras are now read one at a time instead of with `readAll()`, so a new left frame can't make the script reprocess an old front frame (the repeat problem in §4).

**Recording to make (hand-rolled, car centred, 0 → 2 → 4):**
1. Copy `lane_camera.py` to the car.
2. Optional, to get the pose for each frame: start `vehicle_control.py` with the motors off, or `check_pose.py`, from node 0 (§7: start at node 0).
3. `python -u lane_camera.py --save-raw --udp-port 0`, then roll the car slowly, centred, from node 0 through 0 → 2 → 4. Ctrl+C.
4. Copy back `raw_frames/<folder>/`, `lane_camera.csv` and the pose log, if there is one.

**Check first:** the `proc_ms` column and the frame rate printed in the console. The left camera and the JPEG writes add CPU (§4). Not a problem for a hand-rolled pass, but check before recording during a real drive.

## 17. First hand-rolled recording (2026-11-17)

Logs: `logs/car A/raw_frames/20261117_190232/` (166 front, 165 left frames, 23 MB), `logs/car A/lane_camera.csv` (498 rows, 37 s). No pose log.

- **The option works.** 13.5 frames/s processed, `proc_ms` median 18 ms (max 25) with the left camera and the JPEG writes on. That's more than the ~10 ms in §5, but the frame rate held.
- **Timeline from the CSV:** parked 0–14 s, moving ~14–27 s, parked 28–37 s.
  - **Start:** `curve` mode (curvature 0.75 /m, width 23.4 cm). The parked starts in §8 and §11 read `both`, so this start was probably not at the same spot or angle.
  - **Moving:** an intersection with white dashes (~15–18 s, `none`/`curve`), then a curve with the yellow line on the left (~19–26 s, mostly `curve`, a few `both` frames).
  - **End:** a straight seen in `edge` mode, offset −0.1 cm. This looks like node 4 / the start of 4 → 14 (§5, §8), but without a pose it isn't confirmed.
- **Left camera: unusable.** The person rolling the car was walking on its left. A shoe or leg fills the middle of almost every moving left frame, and the hand on the car shows at the top. The yellow line is visible only in the bottom corners, as in §1.
- **No pose:** frames can't be placed on the map. The §12 method needs the map curvature at each frame, so it can't be tested properly on this recording.
- **Rough look at the front camera on the curve (~19–26 s):** the right-edge distance extended back to the axle in a straight line varies from −11.6 to −18.2 cm. For a centred car it should be −12.85 cm. This is the expected extrapolation error (§12), not a real offset.

## 18. Second hand-rolled recording, with `check_pose` (2026-11-17)

Logs: `logs/car A/raw_frames/20261117_191853/` (212 front, 211 left frames; it replaced the §17 folder), `logs/car A/lane_camera.csv` (626 rows, 48 s), `logs/car A/check_pose.log`.

- **Timeline from the CSV:** parked 0–13 s, moving ~13–28 s, parked 28–48 s. The images match §17: start, intersection with dashes (~15–18 s), curve with the yellow line on the left (~19–27 s), then a straight in `edge` mode (offset −0.8 cm, heading −1.9°). The two recordings apparently started and stopped at the same spots.
- **Left camera: usable this time.** No legs in view. The yellow arc is visible in the bottom corners of every frame, and a hand shows only at the top (above the road).
- **Front camera at node 0 reads `curve`** (curvature ~0.8 /m, width 23.0 cm), not `both` as on 11-11 (§11). check_pose puts the car at y = −0.10, 10 cm further along than §11, so more of the curve ahead is in view.
- **check_pose: only the start is usable.** The first 6 lines read (0.06, −0.10, −90°), side offset **+5.9 cm**, the same GPS error at node 0 as in §11 (+5.8 cm). After that the localization jumps between wrong solutions, e.g. (−2.56, 1.13), a path near y ≈ 4.5, and finally (−0.8, 1.5–1.8, −91°) while parked. That last one is close to the wrong solution in §7 (−0.88, 1.96, −93°). The log has no timestamps either, so it couldn't be matched to frames even where correct. **Likely cause (not checked):** the person beside the car blocks part of the lidar's view. Both hand-rolled check_pose attempts failed, while driven runs on 0 → 2 had 0 rejections (§6).
- **Road map geometry, 0 → 2** (`hal/products/mats.py`, `SDCSRoadMap`, right-hand traffic): node 0 at (0, 0.13, −90°), node 2 at (1.127, −1.085, 0°), edge radius `outerLaneRadius` = 438 × 0.002035 = **0.891 m**. So 0 → 2 is a straight of ~0.32 m, a **constant-radius 90° left arc** (lane centre 0.891 m, i.e. 1.12 /m), and a straight of ~0.24 m. The yellow line is the inner boundary (radius 0.763 m), the right edge the outer one (1.020 m). 2 → 4 also uses `outerLaneRadius`.
- **What this means:** the §12 method doesn't need a pose for every frame. On the arc the curvature is fixed (1.12 /m at the lane centre). It only needs to know whether a frame is on the straight, the arc, or the transition between them, and the images show that.
- **Route confirmed by the user:** rolled 0 → 2 → 4 and stopped at node 4 (both recordings). 2 → 4 is a straight of ~0.24 m, a 0.891 m left arc, and a straight of ~0.28 m to node 4. The whole pass is ~3.9 m in ~15 s. The final `edge`-mode straight is node 4 (as in §5, §8).

## 19. Offline test: visible boundary + known curvature (2026-11-17)

Script: [qcar/curve_offline_test.py](../../qcar/curve_offline_test.py) (PC only). Output: `logs/car A/raw_frames/curve_test_20261117_191853/` (`curve_test.csv`, `offsets.png`, a bird's-eye plot `bev_<t>.png` every ~1 s).

**Method:** same yellow and edge pixels as `lane_camera.py`, projected to the floor at the fixed scale (0.724), using points up to 60 cm ahead of the axle. Both boundaries are fitted as circles of known radius around one lane centre, so only the offset `d` and the heading are free (§12 proposal, normal shift as in §14). Two models per frame: **straight** (infinite radius) and **arc** (0.891 m left). True offset ≈ 0, since the car was centred by hand (±1–2 cm).

| Stretch | Arc model: d (sd), heading, rms | Straight model: d, heading, rms |
|---|---|---|
| **Arc, edge only** (23.3–26.5 s, 15 frames) | **−1.7 cm (0.5)**, −0.6°, 0.08 cm | +5.3 cm, −23.8°, 0.23 cm |
| Node 4 straight (28.5–48 s) | −9.5 cm, +23°, 0.32 cm | **−0.6 cm (0.3)**, −1.9°, 0.04 cm |
| Parked at node 0 (arc starts ~10 cm ahead) | −1.0 cm, +7.8°, 0.91 cm | +8.9 cm, −20°, 0.77 cm |

- **On the arc the method works.** With only the right edge (55–65 points, 28–55 cm ahead), the 0.891 m arc lies on the points (rms < 0.1 cm, [bev_024.2.png](../../logs/car%20A/raw_frames/curve_test_20261117_191853/bev_024.2.png)) and gives −1.7 ± 0.5 cm with a plausible heading. A straight line gives +5 cm at −24°, which `lane_camera.py` currently flags as `curve`.
- **The correct model always has the clearly lower rms** on pure arc and pure straight frames, so the fit itself shows when the geometry assumption is wrong.
- **Transitions fail with both models.** Example: [bev_022.0.png](../../logs/car%20A/raw_frames/curve_test_20261117_191853/bev_022.0.png), straight at node 2 with the 2 → 4 arc starting ahead: straight d = +10 cm, arc −0.4 cm, both rms 1.3–1.6 cm. The same happens at node 0 (parked), at 13–14.5 s and at 19–23 s. The straights here (0.24–0.32 m) are shorter than the camera's view range (~0.3–0.7 m), so **most frames near nodes see part straight, part arc.** The arc model is the less wrong of the two (offset within ~5 cm), but its heading is off.
- **Intersection markings (§12 risk 1) confirmed** at 17.1–18.2 s: some edge points land on the white dashes ([bev_017.3.png](../../logs/car%20A/raw_frames/curve_test_20261117_191853/bev_017.3.png)), giving rms 4–7 cm. The robust fit still gave ≈ −1.4 cm there, and the high rms flags the frame.
- **Yellow on curves:** mostly absent (inner boundary out of view). Where it returns at transitions it doesn't add a useful constraint without the correct straight/arc model.

**Next options (not started):**
1. **Piecewise geometry:** straight up to a known distance ahead, then the arc. In the car this distance comes from the path and the car's position along it. Offline it can be a third fit parameter, as a stand-in for the GPS. This covers the transitions. *(Tried in §20: the distance can't be fitted from the image, and along-path errors do matter.)*
2. **Left camera (step 3b)** on the same frames: calibrate it on the node 4 straight (front camera there: −0.6 cm, −1.9°), then test it near the axle on the arcs.

## 20. Offline test: piecewise straight/arc model (2026-11-17)

Added to [qcar/curve_offline_test.py](../../qcar/curve_offline_test.py): the lane centre is straight up to `s0` ahead of the axle and then the 0.891 m left arc ("entry"), or the reverse ("exit"). `s0` is searched over −5 … 85 cm, with offset and heading fitted at each value, as a stand-in for the GPS position. New CSV columns `d_piece`, `h_piece`, `rms_piece`, `piece_kind`, `piece_s0`; the plots show the fit in red (every frame from 12.8 to 28.5 s). Runtime 3.7 min for 212 frames on the PC (36 fits per frame).

- **Pure arcs: unchanged.** 23.3–26.5 s: d = −0.5 … −2.2 cm, rms < 0.07 cm, the same as the arc model in §19.
- **`s0` can't be fitted from the image.** On the node 4 straight (28.5–48 s) the fit alternates between the true answer (straight, d = −0.7 cm, heading −1.9°) and "arc up to 30 cm, then straight": d = −6 cm, heading +17°, with the same rms (0.03 cm, [bev_046.7.png](../../logs/car%20A/raw_frames/curve_test_20261117_191853/bev_046.7.png)). The edge is only seen from ~28 cm ahead. Whatever the lane does between the axle and there is invisible, and an arc there looks just like a turned heading. The same ambiguity explains the bad frames at 21–22 s and 27–27.8 s.
- **So `s0` must come from the car's position on the path,** as planned for the car. That also means **along-path errors matter** (correcting §19): if the curve really starts δ earlier or later than assumed, the fitted heading is off by about κδ (3° for δ = 5 cm), and the offset by roughly 1–2 cm per 5 cm of δ.
- **At intersections the right edge isn't a lane boundary.** At node 0 (parked, and 13–14.5 s) the right edge belongs to the intersection and bends away from the lane. It isn't parallel to the yellow line ([bev_000.0.png](../../logs/car%20A/raw_frames/curve_test_20261117_191853/bev_000.0.png), [bev_013.5.png](../../logs/car%20A/raw_frames/curve_test_20261117_191853/bev_013.5.png)), so every model fits poorly (rms 1–2 cm) and gives +5 … +8 cm. Probably the same near node 2 (19–23 s). The yellow line does follow the lane there, so a yellow-only fit could be used near intersections.

**Coverage of the rest of the route** (curvature of `roadmap.generate_path`, per edge):

| Edge | Length | Shape |
|---|---|---|
| 0 → 2, 2 → 4 | 1.95, 1.91 m | straight 0.2–0.3 m, **left** R 0.89 m (90°), straight 0.2 m |
| 4 → 14, 12 → 0 | 2.88, 1.71 m | straight |
| 14 → 16 | 3.09 m | straight 0.82 m, then **left** R 0.68 m for 189° (roundabout) |
| 16 → 18 | 0.73 m | **right** R 0.62 m (57°) straight away |
| 18 → 12 | 1.45 m | straight 0.48 m, **left** R 0.76 m (48°), straight 0.28 m |

- Only 0 → 2 → 4 has been tested (offline). `curve_offline_test.py` has the 0.891 m left radius built in. The method isn't tied to it: in the car, the signed curvature ahead would be read from the path at the car's position, which covers any radius, turn direction or route. The fit needs signed curvature (centre on the right for right turns), a small change.
- **Untested cases:**
  - **Right turn 16 → 18:** the inner boundary is then the right edge, and the yellow line is the outer one. The front camera probably keeps the yellow, as it kept the right edge on left turns, but this hasn't been seen.
  - **Roundabout 14 → 16:** the right edge is broken by the entries and exits; the boundary on the left side hasn't been looked at.
  - **Left lane:** `lane_camera.py` has only been checked in the right lane. In the left lane the yellow line is still on the car's left (§1), but the lane width and the right road edge on curves are unchecked.

**What's needed to finish the transition test:** each frame's position along the path. Hand-rolled check_pose loses its position (§18). Driven runs on 0 → 2 → 4 localize cleanly (§6), and their `pose=` lines carry `t_epoch=`. So the next recording should be **a normal drive** with `use_camera = False`, `use_gps_table = True`, and `lane_camera.py --save-raw --udp-port 0`, so that `s0` and the curvature can come from the logged pose. A full lap is likely blocked by the 4 → 14 localization failure (§6), so:
- **Drive 1** (main): start at node 0 as usual. It records 0 → 2 → 4 and stops on 4 → 14 as usual.
- **Drive 2** (optional): `nodeSequence = [14, 16, 18, 12, 0]`, car placed at node 14 (`initialPose` follows the first node). It records the roundabout, the right turn and 18 → 12. Localization past node 14 hasn't been checked since the recalibration. The true offset isn't known in a drive, so the checks would be whether the camera offset stays continuous through the straight/arc changes, and how it compares with the GPS and the table.

## 21. Driven recording, drive 1 (2026-11-17): no usable pose

Logs: `logs/car A/vehicle_control.log`, `lane_camera.csv` (674 rows, 52 s), `raw_frames/20261117_200303/` (228 front frames). Table on, camera not steering, start at node 0. Stopped by hand at node 4.

- **No `pose=` lines.** At the start the GPS read (0.09, −0.10), ~0.25 m from the path's first waypoint, so `roadmap.initial_check` didn't count the start node as reached. The whole drive ran on `initSteeringController`, and only the main controller printed `pose=` lines, the only ones with `t_epoch`. The `GPS:` lines have no time and an uneven rate, and the car paused for 13 readings near (0.08, −0.20). Matching by count would be off by more than the ~5 cm the curve-start test needs (0.25 s ≈ 5 cm at 0.2 m/s). **So the transition test (§20) can't be done on this drive.**
- **Logging fixed** in [qcar/vehicle_control.py](../../qcar/vehicle_control.py) (no change to driving): `GPS:` and `GPS rejected:` lines end with `t_epoch=`, and `pose=` lines are also printed while the start controller is active (marked `init=1`).
- **Other observations:**
  - **Right swing at the start:** the GPS heading went −90° → −101° and the front camera looked into the intersection at 32.5–35 s, the same right swing as in §8. With the table on, the GPS still put the car ~8 cm left of the path at node 0 (x = +0.08 to +0.09).
  - **Near node 4** the camera reads +1 … +5 cm (left), and the GPS x ≈ 2.23 against the path at 2.255 also puts it ~2.5 cm left.
  - **The left camera** shows the yellow line clearly throughout (no person beside the car).
  - GPS rejections on the 0 → 2 curve (8 in a row at about (0.5, −1.4), jumps of ~0.4 m) and 2 on 2 → 4, but the run continued.

**Next:** repeat drive 1 with the new `vehicle_control.py` (copy it to the car), same settings.

## 22. Driven recording, drive 2 (2026-11-17): crossed the divider on the 0 → 2 curve

Logs: `logs/car A/vehicle_control.log` (new format, 14:13), `lane_camera.csv` and `raw_frames/20261117_201313/` (same run). `raw_frames/20261117_201140/` is an earlier run whose logs were overwritten. Settings as in §21: table on, camera not steering. The user reported the car repeatedly crossing into the other lane. **No control settings changed** (checked: Stanley 1.5, trim −0.14, `v_ref` 0.2, table on, camera off. The §21 edit only adds prints after the steering command).

Sequence (times = `t_epoch` − 1794946400 s):
- **17–23 s:** start-phase controller (`init=1`, start ~0.23 m past node 0 as in §21), with the same right swing (−90 → −102°). Then along 0 → 2, `cross_track` +0.02 … +0.06 m (GPS: car slightly right of the path).
- **Already before the failure, the GPS + old table was ~10 cm off here.** At 23.5–24.6 s the camera sees the yellow line only 5.7–6.5 cm from the axle centre (centred would be 12.85 cm), so the car was ~6 cm *left* of centre, while the GPS said 4–6 cm *right*.
- **23.3–24.1 s:** 5 GPS readings rejected, all at ≈ (0.53–0.63, −1.41 … −1.45), 0.34–0.45 m in −y from the estimate.
- **24.34 s:** one more reading of that false solution, (0.67, −1.40), was **accepted**: the dead-reckoned estimate had drifted to within 0.3 m of it. The estimate jumped to (0.72, −1.41) and `cross_track` to **+0.36 m** (car believed 36 cm right of the path), so the controller **steered hard left**.
- **Camera, same moment:** at 24.8–25.3 s `both`, offset **+8.1 → +10.6 cm**, yellow line 4.9 → 2.2 cm from the axle centre (the car is 19.5 cm wide), so the left wheels were over the divider. From 25.6 s the yellow line is out of view to the left (car in the other lane).
- **24.5–26.4 s:** the correct GPS readings (≈ (0.80–1.14, −0.91 … −1.04), on the path) were now **rejected**, because they were 0.4–0.5 m from the wrong estimate. After 2 s with none accepted, the safety stop ended the run at (1.07, −1.42).

**Cause:** a **false localization solution on the 0 → 2 curve**, about 0.4 m in −y around (0.5–0.65, −1.45). The same one appeared in drive 1 (§21: 8 rejections at (0.43–0.66, −1.38 … −1.47)), which survived only because no false reading was accepted. On 11-11, after the recalibration, 0 → 2 → 4 had 0 rejections (§6). So it is new since then, and repeatable at the same place.

**Weakness in the jump check:** once a single false reading is accepted, the estimate is wrong, and every correct reading after it fails the jump check. The check then locks onto the false solution instead of protecting against it.

---

# Part E: 2026-11-17, new direction: the camera steers

**Decided with the user:** the camera steers (offset and angle to the lane, on straights and curves), and the GPS only gives the route position (which curve comes next, turns at intersections). No more `gps_corrections` table; recalibrate only when the GPS is grossly wrong. Plan: [camera_lane_correction.md](../plans/camera_lane_correction.md), "Direction change", steps C1–C4.

## 23. C1: can the camera see where a curve starts or ends? (first look)

Script: [qcar/curve_start_test.py](../../qcar/curve_start_test.py) (PC only). Per frame, the yellow line and the right edge are fitted **separately** with four lane shapes: straight, arc (0.891 m left), entry at `s0` and exit at `s0`, with `s0` limited to the visible range 25–65 cm. A change point counts as **seen** when the best entry/exit fit has under half the error of both pure shapes. The yellow line is reduced to **one centre point per image row**: with all its pixels the fit error was ~0.4 cm from the line's width alone, and with the centre line it is 0.05–0.2 cm.

**Data:** the hand-rolled recording (§18) and drive 1's frames (§21) were no longer in `logs/car A/raw_frames/` (replaced when new folders were copied in; the §19/§20 output folder went with them). The two remaining folders are drives that went wrong: `20261117_201140` (earlier run, no log; it started on the straight before node 0 and crossed left near the 0 → 2 exit) and `20261117_201313` (drive 2, §22). Output: `logs/car A/raw_frames/curve_start_<folder>.csv`.

- **Parked on the straight before node 0** (201140, 29–30 s): both boundaries read straight, d ≈ 0, heading −1°, and agree.
- **Curve entry seen on the right edge** (201140, 30.9–31.3 s): `s0` = 48 → 43 → 40 cm in 0.44 s, about the distance the car covers (a 2–5 cm step every 0.22 s), with rms 0.03–0.05 cm against 0.1–0.2 cm for the pure shapes. At the same time d ≈ −8 cm and heading ≈ −11°: the right swing at the start.
- **Curve exit (0 → 2) seen on the right edge** in both drives (201140 39.3–39.5 s, 201313 30.4–30.6 s): `s0` = 33–35 cm, rms 0.03–0.06 cm. The yellow line there fits "straight, heading −20 … −27°". That is the ambiguity from §20, not a contradiction: the yellow is only seen beyond the change point. A few frames later, on the straight, both boundaries agree within 1–2 cm.
- **The change point is visible for only 2–3 saved frames** (0.4–0.7 s at 5 frames/s). On the car `lane_camera.py` processes ~13.5 frames/s, so ~6–9 measurements per change point.
- **Unresolved at node 0** (201313 parked at the user's node 0 spot, y ≈ −0.10): the yellow line fits "entry at 43–45 cm, heading −16 … −20°" while the edge fits "arc, heading +12 … +14°". For a parked car both headings are implausible and they disagree, so here the paint doesn't match the map's shape (straight 0.32 m + 0.891 m arc), or one boundary isn't a lane boundary.
- **Much of the 0 → 2 curve is useless in these drives:** no boundary for 1–2 s, then edge fits with rms 2–7 cm, while the car was already off its lane.

**Needed to finish C1:** the hand-rolled recording `20261117_191853` and drive 1's frames `20261117_200303`. Both should still be on the car, in `raw_frames/`. They cover 0 → 2 → 4 with the car in its lane, including the 2 → 4 entry from the node 2 straight and the 2 → 4 exit at node 4.

## 24. C1 result on the in-lane recordings (2026-11-17)

The hand-rolled (`20261117_191853`, car centred, true offset ≈ 0) and drive 1 (`20261117_200303`) frames were copied back from the car. Same script and settings as §23. Output: `logs/car A/raw_frames/curve_start_20261117_191853.csv`, `curve_start_20261117_200303.csv`.

**Curve ends (arc → straight): seen every time, on the right edge.**

| Exit | Recording | `s0` seen (cm, every 0.22 s) | Offset d at those frames | Truth |
|---|---|---|---|---|
| 0 → 2 | hand-rolled 19.1–19.6 s | 35, 33, 33 | +2.7, +1.3, −0.6 cm | ≈ 0 |
| 2 → 4 | hand-rolled 26.5–27.4 s | 43, 38, (–), 33 | −0.6, −0.8, −5.6 cm | ≈ 0 |
| 0 → 2 | drive 1 37.8–38.2 s | 33, 33, 33 | +1.6, +0.7, +0.3 cm | unknown |
| 2 → 4 | drive 1 47.0–49.1 s | 40, 35, 38, 33, 35 … | +2.1 … −1.5 cm | unknown |

- **First seen at 40–45 cm, lost below ~33 cm** (the nearest edge points are ~28–30 cm ahead). That gives 2–4 saved frames (0.4–0.9 s), about 6–12 frames at the car's 13.5 frames/s. Scatter ±3 cm in `s0`.
- **Once the change point is under ~33 cm, the fit becomes ambiguous again** (§20). For example, drive 1 at 48.1–49.1 s keeps `s0` = 35 cm while the heading drifts +8 → +14°. From there the wheel encoder has to carry it.

**Curve starts (straight → arc) next to intersections: not usable.**
- **0 → 2 at node 0** (hand-rolled 12.7–14.7 s, drive 1 31.1–31.6 s, drive 2 parked §23): the yellow line fits "entry at 43–58 cm, heading −17 … −29°", and the edge "exit at 43 cm, heading +6 … +16°". Both headings are implausible for a centred car, they contradict each other, and `s0` doesn't shrink as the car moves (yellow 45 → 58 cm while rolling forward).
- **2 → 4 from the node 2 straight** (hand-rolled 21.5–22.9 s): yellow "entry at 45–53 cm", again not shrinking, with d +2 → +12 cm and heading −10 → −30°. The edge fits "arc, heading +21 → +7°".
- **Likely reason:** both curve starts sit right after an intersection (node 0, node 2). There the lane isn't a constant-width band along the map path. The edge belongs to the intersection, and the lane is wider: ~30 cm at 35–50 cm ahead at node 2 (§19, `bev_022.0`). So the map's shape doesn't describe the paint there. The 0.891 m arc itself matches well once the car is in it.

**Inside the arcs and on the straights the camera is fine** (hand-rolled, edge): 2 → 4 arc, pure-arc fits (23.1–26.3 s) d = **−1.7 cm (sd 0.6)**, heading −0.5°, as in §19. On the node 4 straight d = 0.0 cm (sd 0.3). In drive 1 the yellow and edge pure-arc fits on 2 → 4 (41.8–43.9 s) agree within ~2 cm.

**What this means for the design (C2/C3):**
- **Curve ends:** take `s0` from the camera when it's seen (40–33 cm ahead), then count it down with the wheel encoder.
- **Curve starts after intersections:** don't fit through them. Steer from the last good straight measurement before the intersection, carried by encoder + gyro for ~0.5 m, until a clean pure-arc fit appears (arc model wins with rms < 0.1 cm). On the hand-rolled pass that was ~2 s after the last straight reading at node 2 (21.0 → 23.1 s). At 1° heading error, 0.5 m of dead reckoning drifts ~1 cm.
- **Start the car on the straight before node 0** (as in 201140), not at node 0 itself, where neither boundary can be used.

**C2 needs the distance travelled per frame**, which none of the recordings have: `lane_camera.py` doesn't read the encoder, and only drive 2 has timed GPS. Proposed: a small logger that reads only the QCar's encoder and gyro (no GPS, no motors) with `t_epoch`, run alongside `lane_camera.py --save-raw` during another hand-rolled, centred pass from the straight before node 0 through 0 → 2 → 4.
**Written 2026-11-17:** [qcar/odometry_log.py](../../qcar/odometry_log.py) → `odometry.csv` (`t_epoch`, encoder counts, distance from encoder and tach, gyro z, heading). The gyro bias is averaged over the first 3 s with the car still; throttle and steering are held at 0. Compiles on the PC; not yet run on the car.

## 25. C2: hand-rolled pass with odometry (2026-11-17)

Logs: `logs/car A/odometry.csv` ([qcar/odometry_log.py](../../qcar/odometry_log.py), first run), `lane_camera.csv`, `raw_frames/20261117_204844/` (214 front frames). Hand-rolled, centred (true offset ≈ 0), from the straight before node 0 to node 4. Script: [qcar/c2_offline_test.py](../../qcar/c2_offline_test.py) (PC only; output `logs/car A/raw_frames/c2_*.csv/.png`).

**The odometry works:** encoder and tachometer distance agree (4.097 vs 4.099 m); moving 45.8–64.6 s; heading +178.9° at node 4. The gyro heading profile matches the map: the 0 → 2 turn spans 0.85–2.3 m (1.45 m, map 1.40 m), and the straight through node 2 is ~0.45 m (map 0.45 m).

**Method (emulates the car without GPS):** position along the path `s` = start + encoder distance. At each frame, the lane shape is taken from the map path (`[12, 0, 2, 4, 14]`) at `s`, any curvature profile. Offset and heading are fitted separately to the yellow centre line and the right edge. An offset is used when rms < 0.15 cm; the camera heading only when rms < 0.10 cm and |h| < 8°, otherwise the gyro carries it. Without a usable fit, dead reckoning uses the encoder + gyro.

**1. The position along the path is the key input.** The start position from matching the gyro heading to the map (−0.45 m, plus 0.13 m to the front axle) gave poor camera fits. Shifting `s` by −0.4 … +0.7 m: the number of clean edge fits with |heading| < 5° rises from 15 (no shift) to **45 at +0.35 m**, then falls again. So the camera's view of the lane matches the map best with the front axle ~0.48 m ahead of the gyro reference point. The wheelbase (0.256 m) explains only part of this.
- **Likely second cause (not checked):** the camera's **forward** distance scale was never verified. Only the sideways scale is corrected (by the lane width, ×0.724). If forward distances are underestimated, the camera sees curve changes closer than they are, which pushes `s` the same way.

**2. Baseline with `s` right (shift +0.35 m, no curve-end corrections):**

| Stretch (s, m) | Source | Offset |
|---|---|---|
| Approach to node 0 (0.22–0.41) | edge | −1.2 … −1.8 cm |
| **First part of 0 → 2 curve (0.47–1.29)** | none: edge rms 0.4–7 cm (intersection markings), yellow out of view | dead reckoning fails (below) |
| End of 0 → 2, node 2 straight, all of 2 → 4 (1.33–3.05) | edge (yellow too where visible) | **−2.3 … +3.1 cm** |
| End of 2 → 4 (3.11–3.5) | edge | −5.8 … −1.4 cm |
| End of 2 → 4 and node 4 straight (3.53–4.09) | edge | **−0.8 … +0.7 cm** |

- **The start of 2 → 4 works** with `s` known, unlike the free fit in §24. Only the start of 0 → 2 stays unusable for the front camera, over ~0.8 m.
- **Dead reckoning over that 0.8 m fails:** the gyro heading relative to the path drifts −6 → −35°, and the offset to −39 cm. The relative heading is the gyro minus the map heading at `s`. With `s` 0.48 m ahead of where the car's body actually turns, the map heading turns ~0.48 m × 1.12 /m ≈ 31° too early, exactly the drift seen.

**3. Curve-end corrections are not robust yet.** With a wide search window (0–1 m ahead), the "curve end seen" test also fires on junk frames (markings in the 0 → 2 curve). The corrections jump by up to ±0.5 m, even when starting from the best position. They need consistency checks (several frames agreeing, small corrections once locked).

**Open, in order:**
1. **Check the camera's forward distance scale** (ruler/tape at known distances ahead, one snapshot). It affects the curve-end distances, the position along the path, and the bridge heading.
2. Make the curve-end correction robust (consecutive frames must agree; limit the step once locked).
3. Bridge the start of 0 → 2: gyro heading with the body/axle lag accounted for, or **the left camera**, which saw the yellow line clearly there in drive 1 (§21).

## 26. Forward distance check (2026-11-17): contradicts the curve geometry

Image: `logs/car A/forward_check_front.png` (`csi_snapshot.py forward_check`). Three paper strips across the lane on a straight, at **30, 45 and 60 cm** from the front axle (user's ruler). Strip rows in the undistorted image at the centre column: 318, 273, 255.5.

| Ruler | Raw projection (`to_ground`) | ×0.724 projection (used by the offline scripts) |
|---|---|---|
| 30 cm | 54 cm | 42 cm |
| 45 cm | 81 cm | 61 cm |
| 60 cm | 105 cm | 79 cm |

- The raw distances fit **true = 0.588 × raw − 1.8 cm** within 1 cm. The strips are 15 cm apart, but the ×0.724 projection puts them 19.5 and 17.4 cm apart.
- **The vanishing point of the lane lines is at row 202.8** (pitch −2.72°), the same as the calibration (−2.67°). The camera pitch hasn't changed.
- **Fitting the camera model to the strips + lane width:** with pitch, height and camera position free, the best fit is a camera 8.3 cm high and 4.8 cm *behind* the front axle (impossible, §9). With the lens height fixed at 10 cm, pitch from the vanishing point, and the two focal lengths + camera position free: **FX 395, FY 289** (borrowed: 496, 455), camera **3.6 cm ahead** of the axle, pitch −4.27°. This fits the strips to 0.2 cm (29.6 / 45.8 / 59.6 cm) and gives the lane width as 25.7 cm at all distances. Added as `lane_camera.to_ground_cal()` (not used anywhere; `to_ground()` unchanged).
- **But it contradicts the curves.** With that projection the C2 curve fits get worse (median |heading| ~6° vs ~1.5°): the forward distances are compressed, so the painted 0 → 2 / 2 → 4 curves look much tighter than the 0.891 m radius. **That radius is confirmed independently** by the encoder + gyro (§25: 90° over 1.45 m, R ≈ 0.92 m). The ×0.724 projection fits the curves well, but not the strips.
- **An encoder cross-check** (the camera's distance to a curve end should shrink exactly as fast as the encoder counts) was inconclusive: the ×0.724 projection gave a slope of −0.40 (expected −1) on noisy data at the edge of the search range, and the new projection found no curve ends.
- **Likely cause:** the fisheye model is borrowed from another calibration ([image_interpretation.py:290](../../skills_activities/02-image_interpretation/image_interpretation.py#L290)). With the wrong intrinsics the undistorted image isn't a true pinhole image, so one linear correction can't fit the centre column (strips), the edges (lane lines) and the curves at the same time. A mistake in the strip measurement is also possible.
- The offline scripts were switched back to the ×0.724 projection, so §25 stays reproducible.

**Follow-up (same day):**
- **Strip measurement:** the user measured from the front wheels. Where exactly doesn't matter for the conflict, which is in the strip *spacing* (15 cm).
- **The calibration script is a template:** `image_interpretation.py`'s `camera_calibration` has no calibration code (it fills in placeholders) and needs a screen.
- **Its comments give Quanser's reference CSI intrinsics** at 820×410 (FX 318.86, FY 312.14, CX 401.34, CY 201.50; distortion −0.9033, 1.5314, −0.0173, 0.0080, −1.1659). The values `lane_camera.py` borrowed are the template's input section, not these. Tested on the strip image: the reference values warp it into a bubble, so they don't fit this camera either.
- **Written:** [qcar/calib_capture.py](../../qcar/calib_capture.py). It saves front-camera frames over SSH (no screen) and reports whether the board is visible. Detection and calibration are done on the PC.

**Next:** a **checkerboard calibration** of the front CSI camera at 820×410 with [image_interpretation.py](../../skills_activities/02-image_interpretation/image_interpretation.py) (`camera_calibration`, front CSI). Then repeat the strip check with the new intrinsics: strips, lane width and curve radius must all agree before C2 can be finished.

## 27. Checkerboard calibration, first recording (2026-11-18)

Frames: `logs/car A/calib_frames/20261118_203057/` (119 frames from [qcar/calib_capture.py](../../qcar/calib_capture.py), car on a desk). Board: letter paper, 3.4 cm squares, **7 × 4 inner corners** (half squares at the ends are ignored by the detector, checked on a zoomed frame). Solver: [qcar/calib_solve.py](../../qcar/calib_solve.py) (PC only), output `calib_frames/calib_result_20261118_203057/`.

| | FX | FY | CX | CY | rms |
|---|---|---|---|---|---|
| Borrowed (`lane_camera.py` now) | 495.8 | 454.6 | 408.0 | 181.2 | – |
| New, standard model (K + 5 coeff.) | 386.8 | 257.2 | 412.8 | 202.1 | 0.19 px |
| New, fisheye model (K + 4 coeff.) | 386.0 | 256.7 | 411.1 | 202.8 | 0.19 px |
| Strip fit, §26 (`to_ground_cal`) | 395 | 289 | – | – | – |

- Board found in 97 of 119 frames; 1 dropped as an outlier. Both models agree.
- **The pixels aren't square: FY/FX = 0.665**, i.e. (4/3) / 2. The 4:3 sensor image is squeezed into the 2:1 820×410 frame, so vertical pixels cover 1.5× the angle of horizontal ones. The borrowed values (FY/FX = 0.92) miss this, which fits the §26 conflict: the strip fit had already found FX 395, FY 289 on its own.
- **CY 202 matches the vanishing-point row (202.8, §26),** so the camera pitch is close to 0 in the new model, not −2.7°.
- **Coverage is the weak point:** all corners lie in the middle of the image (columns ~210–610, rows ~125–300). The bottom quarter and the sides have none. The lane lines are seen exactly there (below the horizon at row ~203, out to the image edges), so the distortion there is extrapolated. The undistorted images look straight in the middle; the edges can't be judged from these frames.
- **Not yet used in `lane_camera.py`.**

**Next:** a second recording with the board held **low in the image** (bottom edge and bottom corners), at the left and right edges, and closer to the camera. Then solve both folders together (`calib_solve.py <folder1> <folder2>`), and if the numbers hold, run the strip / lane width / curve check (§26) with them.

## 28. Second recording and combined calibration (2026-11-18)

Frames: `logs/car A/calib_frames/20261118_204943/` (94 frames, board held lower, closer and further to the sides). Output: `calib_result_20261118_204943/` and `calib_result_20261118_203057+20261118_204943/`.

- **The SB fallback detector was wrong on this board.** On the close-up frames it found "7 × 4" grids that skip a row of squares (the board has full squares plus half squares at the edges), giving 9–11 px errors. These were exactly the frames that reached the bottom of the image. The classic detector was right in every frame checked. **SB is now off by default** in `calib_solve.py` (`--sb` to turn it back on).
- With the classic detector only, 59 of the 94 new frames have the board.

| Solve | Frames | FX | FY | CX | CY | rms |
|---|---|---|---|---|---|---|
| Recording 1 (§27) | 96 | 386.8 | 257.2 | 412.8 | 202.1 | 0.19 px |
| Recording 2 | 57 | 388.9 | 258.3 | 410.2 | 201.2 | 0.19 px |
| **Both, standard model** | **147** | **388.89** | **258.50** | **411.34** | **201.73** | **0.19 px** |
| Both, fisheye model | 147 | 389.4 | 258.8 | 411.4 | 201.5 | 0.19 px |

**Combined standard model:** `DIST = [-0.38827, 0.24861, -0.00028, 0.00010, -0.10079]`.

- The two recordings agree within ~2 px on every term; the standard and fisheye models fit equally well, so the standard model (already used by `lane_camera.py`) is enough.
- **Coverage is still thin at the bottom quarter and the sides** (≤ 11 corners per region). But with these numbers and the camera ~10 cm high, the lane code's working range (28–70 cm ahead, ±13 cm sideways) falls in rows ~205–300 and columns ~240–580, which **is** covered (1000+ corners per region). The bottom quarter is the floor closer than ~25 cm, not used. Outside the covered area the model isn't trusted (one far corner pixel undistorted to nonsense with the recording-2 solution).
- **Not yet used in `lane_camera.py`.**

**Next:** the road check from §26 with the combined numbers: re-project the strip image (30 / 45 / 60 cm), the lane width (25.7 cm, without ×0.724) and the C2 curve fits (0.891 m arc). The camera height, forward position and pitch are re-fitted with the new intrinsics (pitch from the vanishing point, row 202.8 ≈ CY, so close to 0°).

## 29. Road check with the new calibration (2026-11-18): passes

Script: [qcar/road_check.py](../../qcar/road_check.py) (PC only, existing images, nothing new recorded). The §28 intrinsics, with the camera height, pitch and position fitted on the strip image.

**1. Strip image.** With the new intrinsics the undistorted strips are straight (with the borrowed ones they were bent).
- Pitch from the lane lines' vanishing point (row 204.3): **−0.56°**.
- Camera height from the lane width alone: **10.5 cm**. Ruler: 11 cm to the top of the camera, lens ~1 cm lower.
- Strips at **29.8 / 46.0 / 59.2 cm** (ruler 30 / 45 / 60), with the camera **6.2 cm ahead of the front axle** (the unchecked ruler value was 10 cm). The errors are about one image row (0.2 / 0.6 / 1.0 cm per row at the three strips). A free fit of all three parameters gives the same (10.3 cm, −0.79°, 6.1 cm, worst residual 1.0 cm).

**2. Lane width.** Yellow line and road edge are parallel (+1.2° each) and 25.7 cm apart from 25 to 60 cm ahead, **without the ×0.724 factor**. On the C2 pass (§25), 44 straight frames read **26.2 cm** (median).

**3. Curves: the new projection gets the radius right, the old one didn't.**
- A free circle fitted to the right edge on clean pure-arc frames of 2 → 4 (rms ≤ 0.05 cm): **new R = 0.95–1.07 m**, old (×0.724) **1.24–1.51 m**. Map: 1.020 m for the edge. So §26's conclusion was backwards: the ×0.724 projection flattened the curves, and the strips were right.
- **The circle centre is ~0.24 m behind the front axle.** That's the turn centre level with the rear axle (wheelbase 0.256 m). So on an arc, a car that follows the lane points **about −16°** relative to the lane at the front axle (th(s − L) − th(s)), not 0°. The C2 checks (§25, §26) required |heading| < 5°, which only "worked" because the old projection hid the curvature. `road_check.py` now compares with the expected heading.
- C2 pass, new projection: median edge offset **−0.1 cm** (true ≈ 0) at every position shift. Fits matching the expected heading within 3° rise with the shift, up to 32 frames at +0.55 m (old projection: at most 15). This is the along-path position question for C2. The geometry alone predicts ~+0.13 m (front axle = gyro reference + L, not + L/2), so ~0.4 m is still unexplained.
- Where yellow and edge both fit cleanly (≤ 10 frames), they imply a lane only 19–24 cm wide. All of these are near node 0, the node 2 straight, or the start of 2 → 4, where one boundary isn't the lane boundary (§20, §24). The plain straights give 26.2 cm (above).

**New floor projection (not yet in `lane_camera.py`):** K and DIST from §28, `PITCH = −0.56°`, `CAM_H = 0.105 m`, `CAM_AHEAD = 0.062 m`, no `SCALE`.

**Next:** put this projection into `lane_camera.py` (replacing the borrowed K/DIST, pitch, height, `CAM_AHEAD`, and the ×0.724 `SCALE`), so that the car and the offline scripts use it. Then continue C2 with it: the along-path position (shift) and the curve-end corrections.

## 30. New calibration in `lane_camera.py` (2026-11-18)

[qcar/lane_camera.py](../../qcar/lane_camera.py) now uses the §28/§29 calibration:
- K = FX 388.89, FY 258.50, CX 411.34, CY 201.73; DIST from §28.
- `PITCH` −0.56°, `CAM_H` 0.105 m, `CAM_AHEAD` 0.062 m.
- `SCALE` = 1 (the name is kept so older code still runs).
- `FIT_MAX_FWD` 0.55 m from the camera, ≈ 0.61 m from the axle, the same reach as before.
- `to_ground_cal()` (§26) removed.
- Horizon row 204.3, `ROI_TOP` 222 (was 220).

The offline scripts take the projection from `lane_camera.py`, so they now use it too. Results in §19–§26 were made with the old one. `c2_offline_test.py` caches its points per calibration (`c2_points_<folder>_fx389.npz`). `road_check.py` keeps the old projection as `K_OLD`/`OLD` for comparison.

**Check on the C2 pass (`20261117_204844`, all 214 frames, new code vs the CSV the car logged with the old code):**

| | Old (logged) | New |
|---|---|---|
| Modes | both 101, curve 59, edge 50, none 4 | both 96, curve 64, edge 50, none 4 |
| `both`: offset (car centred) | +0.1 cm | +0.1 cm (sd 0.2) |
| `both`: lane width | 25.9 cm (×0.724) | 26.5 cm (true scale) |
| `edge`, node 4 straight | −0.8 cm | −0.6 cm |

- Straights read the same as before. The lane width is 3 % high at the node 0 approach (the strip image gave 25.7 cm on another straight). That's well inside `WIDTH_TOL`, and the `both` mode rescales by the measured width anyway.
- A few more frames are flagged `curve`, because curvature is now at true scale. `lane_camera.py` still doesn't steer on curves; that's C2.
- On the strip image: `both`, offset +0.3 cm, width 25.8 cm, confidence 1.0.

**To use it on the car:** copy the new `lane_camera.py` to car A. Nothing else changes (UDP format, CSV columns, `vehicle_control.py`).

**Next:** C2 with the new projection: the along-path position (§29: best shift ~+0.5 m vs ~+0.13 m expected from the geometry) and robust curve-end corrections (§25 open item 2).

**Correction (2026-11-18):** `vehicle_control.py` on the PC has `use_camera = False`, `use_gps_table = True` (table mode, the safe setting), not camera steering as stated in §13/§15.

## 31. C2 rerun with the new calibration and the rear-axle geometry (2026-11-18)

[qcar/c2_offline_test.py](../../qcar/c2_offline_test.py) changes (same recording and odometry as §25):
- **Expected heading.** A car following the lane points `h_exp = th(s − L) − th(s)` relative to the lane at the front axle (−16.4° on the 0.891 m arcs, 0 on straights; §29). The camera heading is accepted within 8° of `h_exp`, not of 0.
- **Dead reckoning.** The offset changes with `h − h_exp`, not with `h`. On an arc the body moves along the lane when `h = h_exp`; the old code read that as a −16° heading error.
- **Start position.** The gyro/map alignment matches the body heading, which is the lane direction at the rear axle. So the front axle is the reference + L (0.256 m), not + L/2.
- Output: `c2_<folder>_fx389[...]` (the §25 files are kept).

**Result, curve-end corrections off (`nofix`), no extra shift:** the whole pass from the straight before node 0 to node 4 stays within **±5.8 cm** (mean −0.5 cm, sd 1.9; true ≈ 0). Plot: `raw_frames/c2_20261117_204844_fx389_nofix.png`.
- **The heading follows `h_exp`** through both arcs. Camera headings on 2 → 4 are −12 … −18° against −16.4° expected.
- **The 0 → 2 curve is bridged by dead reckoning for ~1.6 m** (s 0.1–1.75: the front camera has no clean fit there, §25). The offset drifted only to −2.7 cm. In §25 the same stretch drifted to −39 cm.
- 2 → 4 is mostly measured by the right edge (blue), within ±1.5 cm, except −5 cm at the arc end (s ≈ 3.3).

**The along-path position: no unexplained shift left.**

| Extra shift (m) | −0.20 | −0.10 | −0.05 | **0** | +0.05 | +0.10 | +0.20 | +0.40 |
|---|---|---|---|---|---|---|---|---|
| max \|d\| (cm) | 24.6 | 10.2 | 5.3 | **5.8** | 11.3 | 19.7 | 16.3 | 23.0 |
| sd (cm) | 7.4 | 3.2 | 1.9 | **1.9** | 4.1 | 6.3 | 4.2 | 5.0 |

- The best start position is the one the car's geometry predicts (shift −0.05 … 0). The ~0.4–0.5 m in §25/§29 came from the old projection plus the wrong expected heading.
- **Requirement:** the position along the path must be known to about **±5 cm**. At ±10 cm the offset errors reach 10–20 cm, mostly in the dead-reckoned stretch.

**Curve-end corrections still harmful** (with them on: two corrections, −0.50 m (clipped) and −0.28 m; offset up to 54 cm). Same as §25: junk frames in the 0 → 2 curve pass the "curve end seen" test.

**Next:**
1. Make the position along the path reliable to ~±5 cm. Options: robust curve-end corrections (several frames must agree, small steps once locked, only near a mapped curve end), and/or the GPS along-path position where it's clean.
2. Then C3: camera steering in `vehicle_control.py` with this method.

## 32. Position along the path from the gyro turn (2026-11-18)

**Chosen approach** (instead of the GPS or the camera curve ends, which are both unreliable here, §22, §25, §31): the encoder moves the position along the path. At each curve it is corrected by matching the gyro heading to the map. The camera curve ends and the GPS are only cross-checks.

[qcar/c2_offline_test.py](../../qcar/c2_offline_test.py), new default mode `gyro` (`cam` = the old curve-end corrections, `nofix` = none):
- Body heading in map coordinates = gyro + `g_off`. `g_off` comes from the camera heading on a straight:
  - while **parked at the start**, from a plain straight-line fit, which doesn't need the position;
  - while driving on mapped straights.
- Every frame, the last 0.5 m of encoder/gyro samples is compared with the map direction at the **rear axle** for position errors within ±0.40 m. This is used only when the map turns by > 15° within the window. The position moves towards the best match by 30 % per frame, at most 3 cm per frame.
- Camera curve ends are only checked (gated: a mapped curve end must be 20–60 cm ahead), not applied.

**Test:** the C2 pass with the start position deliberately wrong by up to ±40 cm. "Truth" = the §31 start position.

| Start error | Position error at s = 1.0 / 1.5 / 2.6 / 3.3 m | max \|pos. error\| after s = 1.2 m | Offset sd / max |
|---|---|---|---|
| 0 | −4.0 / −0.4 / −3.9 / +3.4 cm | 3.9 cm | 1.6 / 5.4 cm |
| +10 cm | −4.5 / −1.1 / −5.1 / +2.7 | 5.1 | 2.1 / 5.8 |
| −10 cm | −5.0 / −0.6 / −3.9 / +3.4 | 3.9 | 1.9 / 5.8 |
| +20 cm | −4.2 / −1.1 / −5.2 / +2.6 | 5.2 | 2.5 / 5.8 |
| −20 cm | −9.0 / −0.7 / −3.9 / +3.3 | 3.9 | 3.4 / 7.5 |
| +30 cm | −2.2 / −1.0 / −5.0 / +2.8 | 5.0 | 3.4 / 8.0 |
| −30 cm | −24 / −1.6 / −4.0 / +3.4 | 4.0 | 5.5 / 14.2 |
| ±40 cm | +34 / −0.8 … −40 / −10 | 5.1 / 10.0 | 5.5–7.9 / 13.6–25.5 |

- **Start errors up to ±30 cm are corrected inside the 0 → 2 curve.** From the middle of that curve on, the position stays within ±5 cm through node 2, the whole 2 → 4 curve and node 4.
- **The offset (the lane-keeping result) stays within ±6 cm for start errors up to ±20 cm.** Larger start errors cost up to 14–25 cm in the first part of 0 → 2, before the correction catches up. So **the start position must be known to about ±20 cm**. The GPS at node 0 was 6–8 cm off sideways (§8, §11, §18), and along the path it should be well within this.
- **The ±3–5 cm that remains inside the arcs is the same in every run**, so it's systematic: the "truth" itself is only known to ~5 cm (§31), or a small gyro offset (1° ≈ 1.6 cm here).
- **First version failed for positive start errors:** `g_off` was only learned with the map shape at the estimated position, which never fitted cleanly when that position was wrong. So the gyro match never started. Learning it while parked fixed this. **For the car: start parked on a straight** (as §24 already recommends) **with the camera in `both` mode.**
- **Camera curve-end checks (gated):** they hardly fire now (0–1 per run), so they add little as a cross-check.

**Limits:** one hand-rolled recording, two left curves of the same radius. Untested: the roundabout (14 → 16), the right turn (16 → 18), driven speed, and a second lap (gyro drift over a longer time; `g_off` is re-learned on every straight).

**Next: C3, camera steering in `vehicle_control.py`** with this method: the encoder + gyro position along the path, corrected at curves; the camera offset/heading from the map lane shape at that position; dead reckoning where the camera is blind; the expected heading on arcs. Start parked on a straight before node 0. The GPS gives only the start position (±20 cm), the route, and a sanity check. Before steering with it, a measure-only drive (table on, camera logged) to compare with the GPS.

## 33. Measure-only drive prepared (2026-11-18, code only, not yet run)

[qcar/vehicle_control.py](../../qcar/vehicle_control.py), logging only (driving unchanged: `use_camera = False`, `use_gps_table = True`, `v_ref = 0.2`):
- `log_odometry = True` writes **`odometry_drive.csv`** on the car every 0.02 s. Columns: `t_epoch`, `encoder`, `dist_m`, `tach_mps`, `dist_tach_m`, `gyro_z`, `heading_deg` (same as `odometry.csv`, so `c2_offline_test.py` reads it directly), plus `gyro_z_raw`, the EKF pose `x`, `y`, `th_deg`, and the steering command `delta`.
- The gyro bias is averaged while parked during `startDelay` (1 s, ~200 samples). `gyro_z_raw` allows redoing it offline.
- Console: `ODOMETRY: logging to …`, `ODOMETRY: gyro bias …`, `ODOMETRY: saved …`.

Procedure: [car_A_setup.md](../car_A_setup.md) §3, "Measure-only camera recording drive". The car is parked on the straight before node 0 with the camera in `both` mode, and `lane_camera.py --save-raw --udp-port 0` runs alongside. The setup doc's configuration table was also out of date (`use_camera = True`, `use_gps_table = False`), now fixed.

**What the drive gives:** the C2 + gyro-position method (§31, §32) on a driven run at 0.2 m/s, compared with the GPS (`pose=` lines and the EKF pose in the odometry file). The true offset isn't known in a drive. The checks are whether the camera offset stays continuous through the curves and the dead-reckoned stretch, and how it compares with the GPS + table.

## 34. Measure-only drive (2026-11-18): camera vs GPS, encoder scale

Logs: `logs/car A/vehicle_control.log`, `odometry_drive.csv` (1202 rows), `lane_camera.csv`, `raw_frames/20261118_222611/` (307 front + left frames). Table steering, camera not steering, `v_ref` 0.2. Parked on the straight before node 0 (EKF start (0.08, 0.50)). Gyro bias +0.0046 rad/s from the parked second. 5.31 m driven in ~28 s. Stopped on 4 → 14 at y ≈ 1.06 (below). Offline: `c2_offline_test.py … odometry_drive.csv 0 nofix 0.98` and `… gyro 0.98`.

**1. Camera offset over the drive (C2 method): continuous, within ±10 cm, and it shows the GPS + table errors.** Camera − GPS cross-track (sign-matched), at the `pose=` lines:

| Stretch | Camera | GPS + table believed | Camera − GPS |
|---|---|---|---|
| Parked at the start | −0.3 cm (both lines) | +7.8 cm | −8 cm |
| Approach and node 0 (right swing, heading −13°) | **−9 … −10 cm** (edge) | −1 … −3 cm | **−6 … −8 cm** |
| 0 → 2 curve (camera blind, dead reckoning −8 → −4 cm) | DR | −2 … +0.6 cm | −3 … −6 cm |
| End of 0 → 2, node 2, 2 → 4 | −4 … +2 cm | −5 … +2 cm | **within ±4 cm** |
| End of 2 → 4, node 4, 4 → 14 (GPS clean) | +1 … +2 cm | −2 … +1 cm | +2 … +4 cm |
| 4 → 14 after the GPS was lost | **+3 → +6 cm** | −1 … −2 cm (EKF) | +6 … +7 cm |

- **At the start the GPS + table was ~8 cm off sideways.** The controller "corrected" it, so the car swung right to −9 cm. This is the start swing of §8, §11 and §21, now measured all the way along the approach.
- After the GPS was lost on 4 → 14, the camera sees the car drift left by ~5 cm in 2 s, as in §8.
- Over 1 m of dead reckoning in the 0 → 2 curve, the offset went from −8 to −4 cm. At the first camera fit after it (s = 1.51), the camera read +0.7 cm, a 4.6 cm step. It's unknown how much of that is drift and how much real movement.

**2. The encoder over-reads distance by 2 %.** While the GPS was accepted (122 readings, 0.07–4.92 m), the GPS position along the path = **0.980 × encoder distance** − 0.36 m, rms 2.4 cm (tachometer: 0.979). The hand-rolled pass agrees: the 0 → 2 turn spanned 1.45 m of encoder distance against 1.40 m on the map (§25). `c2_offline_test.py` takes the scale as an optional 5th argument.

**3. Position along the path: encoder × 0.98 is good; the gyro match is not robust in a drive.** Estimated position minus GPS position, along the path:

| Method | Mean | sd | Range |
|---|---|---|---|
| Encoder only (`nofix`) | +10.0 cm | 4.0 cm | +4 … +19 cm (drifts ~3 %) |
| **Encoder × 0.98 (`nofix`)** | +10.6 cm | **2.3 cm** | +5 … +16 cm |
| Encoder × 0.98 + gyro match (`gyro`) | +7.2 cm | 7.0 cm | −13 … +16 cm |

- The constant ~+10 cm is expected: C2's position is the front axle, and the GPS point is about half a wheelbase (12.8 cm) behind it. So the geometric start alignment and the GPS agree within ~2 cm.
- **The gyro match pulls the position by up to ±15 cm in this drive.** Hand-rolled, the car followed the lane, so its heading matched the map. Driven by the table controller, the car's heading also contains the steering corrections (e.g. the −13° swing at the start, the S-shaped corrections on 2 → 4). The matcher reads those as position errors.
- Tried and reverted: removing the camera's heading-error estimate before matching. It made the hand-rolled pass fail (start errors of ±20 cm no longer corrected), because that estimate itself depends on the position.
- **The GPS position along the path was good while it was accepted** (2–3 cm against the scaled encoder). Its errors on this route are sideways offsets and occasional false solutions, which the jump check catches.

**4. The stop on 4 → 14 was a new false solution.** From y ≈ 0.6 on, every GPS reading put the car **0.6–0.8 m behind** (y 0.0–0.4, around node 4) while the car moved on. In §6 it was x ≈ 3.2 instead. The encoder, the EKF and the continuous camera fits all agree that the car kept moving.

**What this changes for C3** (revises §32's choice):
- **Position along the path:** encoder × 0.98 from the start alignment. Corrected from the **GPS position along the path** whenever a GPS reading is accepted and agrees within ~10 cm (low-pass). The gyro match is only a fallback while there's no GPS, or dropped.
- **Sideways offset and heading:** the camera, using the map's lane shape at that position, with dead reckoning where it's blind.
- **The GPS sideways position isn't used for steering at all.** It's the part that was 8 cm off at the start.
- Both the driven and hand-rolled runs cover only 0 → 2 → 4 → start of 4 → 14.

## 35. C3 written: camera steering (2026-11-18, code only, not yet driven)

Agreed with the user (2026-11-18): the camera does the lane keeping; encoder × 0.98 plus the GPS position along the route give the position along the route; the GPS sideways position isn't used; the gyro match is dropped.

**[qcar/lane_camera.py](../../qcar/lane_camera.py):**
- Receives the map's lane curvature around the car from `vehicle_control.py` on UDP port 18556: "t, s, k at −0.30 … +1.00 m in 5 cm steps".
- Each frame, fits the yellow centre line and the right edge **separately** with that lane shape (only offset and heading free). Same point extraction and selection rules as `c2_offline_test.py`. It's a small Gauss-Newton fit with soft-L1 weights, so it needs no scipy on the car.
- Sends `mf_src`, `mf_d_cm`, `mf_h_deg`, `mf_rms_cm`, `mf_s` with every UDP result. They're also logged in `lane_camera.csv` with `mf_ms`.
- Check against the offline fit on the hand-rolled frames: same source on every frame, offset within 0.3 cm, heading within 0.8°. 6 ms per frame on the PC.

**[qcar/vehicle_control.py](../../qcar/vehicle_control.py), `use_camera_steer` (default `False`):**
- `RoutePath`: the route as a function of the distance s (cyclic; s < 0 is the approach straight before node 0).
- Start: s from the first GPS reading + `gps_front_ahead` (0.11 m). It refuses to start unless the car is on the straight before node 0 (−0.9 < s < 0.15 m, within 15 cm of the route).
- Every loop: s += 0.98 × encoder distance. Offset and heading are carried on with `h − h_exp` and the gyro (bias from the parked second).
- Each GPS reading moves s by 20 % of the along-route difference, only if that difference is < 10 cm and the reading is < 15 cm from the route. This uses its own gate, not the EKF jump check.
- Each camera result: offset from the map-shape fit. The heading too, if the fit rms < 0.10 cm and it's within 8° of the expected heading. While parked, the `both` mode result is used if there's no map fit yet.
- Stanley steers from the front axle point placed at that offset from the route, with that heading. On an arc its heading term gives the ~16° turn by itself.
- Safety stops: no camera fit for 1.8 m, |offset| > 12 cm, no message from `lane_camera.py` for 0.5 s, and the planned stop `cs_stop_s` = 6.6 m (just before node 14; the method is untested beyond). The GPS timeout stop is off in this mode.
- `pose=` lines show `cs: s, offset, heading, src, fits, gps_s`. `odometry_drive.csv` gains `cs_s`, `cs_d_cm`, `cs_h_deg`, `cs_src`.

**Offline checks:**
- **Replay of the §34 drive** ([qcar/c3_replay.py](../../qcar/c3_replay.py), using `RoutePath` and the settings from `vehicle_control.py`'s source):
  - start at s = −0.26 m (the offline alignment gave −0.25);
  - 89 camera fits, 138 GPS along-route corrections;
  - offset mean −1.8 cm, sd 4.3, max 10 cm (as C2 in §34);
  - longest stretch without a fit 0.63 m; no safety stop would have triggered.

  The steering isn't simulated: that car was steered by the table.
- **Kinematic simulation** of the steering loop (`SteeringController` from the file, bicycle model, ideal measurements, start 3 cm left and +3°): it corrects to < 0.1 cm in ~0.3 m. On the arcs it steers +16.4 … +16.9° and holds the offset within 0.1 cm.

**Not covered:**
- Real camera noise in closed loop, and the delay (frame ~20–40 ms plus UDP).
- The CPU load of the fit on the Jetson.
- Everything past node 14.

**Next:** the first camera-steering drive. Procedure: [car_A_setup.md](../car_A_setup.md) §3, "First camera-steering drive".

## 36. First camera-steering drive (2026-11-18): 6.3 m on the camera, stopped by one bad fit

Logs: `logs/car A/vehicle_control.log`, `odometry_drive.csv` (with `cs_*`), `lane_camera.csv` (with `mf_*`), `lane_debug/`, `raw_frames/20261118_225612/`. `use_camera_steer = True`, table not used for steering, `v_ref` 0.2. (The §34 drive's `odometry_drive.csv` and log were overwritten by this run; its frames and results are kept.)

**The car steered itself on the camera from the straight before node 0, through 0 → 2, node 2, 2 → 4, node 4 and 2.3 m along 4 → 14** (s = −0.19 → 6.15 m, ~33 s).
- **Start:** GPS put the front axle at s = −0.19 m, 8.1 cm off the route sideways. The camera said centred (+0.1 cm, both lines), so the car went straight, with no right swing (§34).
- **Past the GPS failure on 4 → 14:** from y ≈ 0.1 every GPS reading was rejected (0.6–1.1 m behind, then the x ≈ 3.2 false solution of §6). The camera steering carried on. Its own along-route gate also ignored those readings (`gps_s` stayed at 133–135).
- **Offset as seen by the camera** (the `pose=` lines):

  | Stretch | Offset |
  |---|---|
  | Approach, node 0 | −1.4 … +0.1 cm |
  | 0 → 2 (mostly dead reckoning; one edge fit −5.5 cm at s ≈ 0.9) | −5.5 … +3.2 cm |
  | Node 2 (bad edge fits, below) | up to +9.2 cm, back to −1 cm within 0.2 m |
  | 2 → 4 arc | **−0.6 … −0.3 cm** |
  | Node 4 and 4 → 14 | **−1.3 … −0.4 cm** |

- On the 2 → 4 arc the camera heading was −19 … −21° against −16.4° expected. Stanley holds the offset at −0.5 cm with that, so the ~3–4° difference is probably real (slip, or the steering trim).
- **CPU on the car:** 13.1 frames/s processed. `proc_ms` median 13.6 ms, map fit median 7.8 ms (max 35 ms). Fine.
- Fit sources: E 246, YE 85, Y 4, none 107 (of 442 frames with a profile).

**Why it stopped (s = 6.15, offset safety stop at −27 cm):** two consecutive map fits read −27 cm, while the straight-line mode read −1.7 cm on the same frames. Near node 14 the roundabout's yellow arc is also in view on the left. The map fit started its edge search from a line through **all** yellow pixels, which was pulled left by that arc, so the search started left of the divider and took the divider as the right edge (25.7 cm too far left).

**The node 2 jump (s 2.13–2.25):** edge fits of +6 … +9.5 cm (clean, rms 0.04–0.14 cm) for 0.12 m: the intersection edge that isn't the lane boundary (§20). The car steered on them briefly.

**Fixes (code only, not yet driven):**
1. `lane_camera.py`: the map-fit edge search starts from the robust (inlier) yellow fit when there is one, and only falls back to the all-pixel fit otherwise. The −27 cm frame now gives −1.1 cm.
2. `vehicle_control.py` **jump gate:** a camera offset further than 4 cm + 3 cm per metre of dead reckoning from the prediction is ignored (`CAMSTEER: camera offset … ignored`). It's accepted anyway if the camera keeps saying the same (sd < 2 cm) for 0.15 m (`re-lock`). Ignored fits don't reset the dead-reckoning distance, so the 1.8 m stop still applies. `pose=` shows `rej=`.
3. `c3_replay.py` has the same gate. Replay of this drive with both fixes: the three node-2 frames are ignored (+9.0, +6.8, +3.3 cm against a predicted −4 cm). No stop. Offset over the run within −4.2 … +4.6 cm (sd 1.2). From 2 → 4 on, within −1.3 … −0.1 cm.

**The 0 → 2 curve: a single bad fit steered the car towards the divider.** The user saw the car touch, or get close to, the divider in the 0 → 2 curve. The logs explain it:
- s 0.42–0.88: no camera fit (the front camera is blind there, §25). Dead reckoning held the offset at +0.4 … +0.5 cm, heading close to the expected −16°. Steering +13 … +15°, i.e. a normal curve.
- **s = 0.90: one isolated edge fit read −5.3 cm (car right of centre), rms 0.133 cm** (just under the 0.15 limit), with no fit before or after it. The controller took it, and its heading (−17.9°) too.
- **s 0.91–1.25: steering at full left lock (+30°) for ~1.5 s,** to "correct" a 5.5 cm right offset. Dead reckoning followed the extra left turn: −5.5 → −2.4 cm by s = 1.44.
- **s = 1.45: the yellow line came back into view, +3.3 cm (left of centre, close to the divider),** confirmed by edge fits (+4.8 … +1.0 cm). This matches what the user saw.
- Consistency check: dead reckoning moved the offset by +3.1 cm between s 0.90 and 1.44. Starting from the pre-fit +0.5 cm instead of −5.3, that ends at ≈ +3.6 cm, the same as the camera's +3.3 cm. So the car was most likely centred before s = 0.9, and the −5.3 cm fit was ~6 cm wrong. Yes, it was a correction, but of an error that wasn't there.
- The new jump gate would only just have caught it (difference 5.8 cm, gate 4 + 3 × 0.48 = 5.4 cm).

**Further fix (made, not yet driven): confirmation after a gap.** In `vehicle_control.py`, after ≥ 0.3 m without an accepted fit (`cs_confirm_after`), a new fit is used only when the **next** fit agrees with it within 2 cm (`cs_confirm_tol`). A frame without a fit breaks the pair. Then the jump gate applies as before. `pose=` shows `wait=`. Also fixed: an ignored fit showed as `DR` instead of `rej` in `cs_src`.

`c3_replay.py` has the same rule, and a new optional 4th argument that replays the **map fits the car actually received** (`mf_*` in `lane_camera.csv`, all ~13 frames/s) instead of recomputing them on the 5/s saved frames. Replay of this drive's message stream with the gate and the confirmation:
- **s = 0.89:** the −5.3 cm edge fit waits for confirmation, gets none, and isn't used. Dead reckoning then follows the car's real (live-steered) left drift. At s = 1.45 it predicts +2.2 cm, where the yellow line reads +3.3 cm. This supports the explanation above.
- **s = 1.45:** the real re-acquisition waits one frame and is accepted.
- **Node 2:** the +6 … +9.5 cm edge fits are all held or ignored, until the camera reads −0.6 cm at s = 2.35, which is accepted.
- **s = 6.14:** the −27 cm fit is ignored (that drive's `lane_camera.py` still had the old edge search).
- Over the run: 316 fits used, 10 held for confirmation, 7 ignored. Offset mean −0.9 cm, sd 1.3, max 5.9 cm (at node 2, dead reckoning). No stop.

**Next:** the same drive again with the fixes (copy `lane_camera.py` and let `vehicle_control.py` upload), still stopping at `cs_stop_s` = 6.6 m.

## 37. Second camera-steering drive (2026-11-18): crossed the divider at node 2; the gate locked onto a wrong estimate

Logs: `logs/car A/vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv`, `raw_frames/20261118_231554/` (they replaced the §36 drive's logs; its frames are kept). With the §36 fixes (edge search, jump gate, confirmation). The user saw the car fine on 0 → 2, then crossing the divider at the start of 2 → 4, then stopping.

**Sequence:**
1. **s 1.54–1.70, the 0 → 2 exit:** edge fits gave offsets +2.9 → +0.2 cm, but **headings −23 … −26°**, where a lane-following car points ≈ −16°. They were accepted (within the 8° allowance at first). Near a curve end the edge fit can't separate offset and heading (§20, §24).
2. **s 1.72–2.11, no map fits:** dead reckoning carried that heading error, so the estimate drifted **−0.4 → −4.0 cm** (right). Stanley read the car as pointing ~9° right of the lane and too far right, and steered **+17 … +26° left on the node 2 straight**, where the curve had already ended.
3. **The car went left.** The straight-line mode saw it, **+0.9 → +6.9 cm** at s 1.92–2.04. Then the map fits read **+8.0 → +12.3 cm** (s 2.12–2.29), the last ones with the yellow line agreeing (YE).
4. **The jump gate ignored all of them** (`camera offset +8.8 … +11.2 cm ignored, predicted −3.9 … −3.0`), because they disagreed with the drifted estimate. It re-locked only after 0.15 m, at +12.3 cm, which tripped the 12 cm offset stop. At +12 cm the left wheels are ~9 cm over the divider.

**This revises §36:** the +6 … +9.5 cm fits at node 2 in the first drive were most likely **real**. That car had drifted left the same way, and it recovered because it acted on them. The gate, added to reject them, made it worse. It has the same weakness as the GPS jump check (§22): once the estimate is wrong, correct measurements are rejected.

**Fixes (code only, not yet driven):**
- **(a) No camera heading near curve starts and ends.** If the map curvature changes by more than 0.3 /m between 0.30 m behind and 0.60 m ahead of the front axle (`cs_head_window`, `cs_head_kchange`), the fit's heading isn't used; the gyro carries it. The offset is still used.
- **(b) Faster re-lock:** 3 consecutive ignored fits within 1.5 cm of each other are accepted (`cs_relock_n`, `cs_relock_n_sd`), as well as the 0.15 m rule.

**Replay of this run's message stream** (`c3_replay.py … lane_camera.csv`; the car's recorded motion can't change, so it still ends at +12 cm):
- **Without the fixes:** at the node 2 straight the estimate says −4.1 cm, heading +0.5° (car right, pointing straight). Re-lock at s = 2.28.
- **With (a):** it says +2.8 cm, heading +10.4° (car left and turning left: the true direction), rising to +4.7 cm at s = 2.19. In the live drive Stanley would then have steered **right** instead of left on the node 2 straight, so the drift would most likely not have started. This can only be confirmed on the car.
- **With (b):** re-lock at s = 2.20, ~8 cm of travel earlier than the 2.29 in the drive.

A closed-loop simulation with an assumed heading error at the curve ends was tried, but its error model didn't reproduce the crossing, so it isn't used as evidence.

**Next:** the same drive again (copy `lane_camera.py`, `vehicle_control.py` uploads), stopping at `cs_stop_s` = 6.6 m. Watch the 0 → 2 exit and node 2.

## 38. Third camera-steering drive (2026-11-18): reached the planned stop

Logs: `logs/car A/vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv`, `raw_frames/20261118_232957/` (they replaced the §37 drive's logs; its frames are kept). All fixes of §36–§37 in, `cs_gate0` 0.04 m.

**The car drove the whole tested stretch on camera steering and stopped at the planned stop** (`STOPPED (camera steering): planned stop at s = 6.60 m`, offset −1.0 cm): from the straight before node 0 (s = −0.29 m), through 0 → 2, node 2, 2 → 4, node 4, and 2.7 m along 4 → 14.
- The GPS failed on 4 → 14 again (50 rejected readings, from y ≈ 0.1). The along-route corrections stopped at 136–138, and the car carried on.
- **Jump gate: never triggered** (`rej=0`). 3 fits were held for confirmation.
- **CPU:** 13.2 frames/s, map fit median 8.1 ms.

| Stretch | Offset estimate | Steering (before trim) |
|---|---|---|
| Approach, node 0 | −2.1 … 0 cm | 0 … +30° (start) |
| 0 → 2 curve | −4.1 … +5.6 cm | −10.6 … +30° |
| Node 2 | −3.3 … −0.4 cm; camera −0.5 … −2.6 cm | 0 … +28° |
| 2 → 4 curve | −2.7 … −0.3 cm | +10 … +26° |
| Node 4 | −0.9 … −0.5 cm | +8 … +22° |
| 4 → 14 | −1.3 … −0.7 cm | +4.5 … +8.5° (≈ −2° net with the −8° trim) |

- **Node 2, where §37 crossed the divider, was clean.** With no camera heading taken at the 0 → 2 exit, the dead-reckoned estimate didn't drift, and the camera fits agreed with it.
- **Remaining weak spot: the 0 → 2 curve around s ≈ 0.9 m** (the same place as the bad fit in §36):
  - two agreeing edge fits said −4 cm, so the confirmation let them through, and the car steered at full left lock (+30°) for ~0.3 m;
  - at s = 1.59 the camera read +4.8 cm, where dead reckoning had moved only to −0.5 cm.

  So camera and estimate disagree by ~5 cm inside this curve. **The user saw the car get close to the divider in the 0 → 2 curve again, but judged it manageable for now.** Most likely the edge fits around s ≈ 0.9 are systematically off: the stretch right after the node 0 intersection, where §24/§25 found the edge unusable.

**Status:** camera steering works over 0 → 2 → 4 → 14 at 0.2 m/s, with one known weak spot in the 0 → 2 curve.

**Open, in order of risk:**
1. The 0 → 2 edge fits around s ≈ 0.9. Option: no edge-only fits in that first part of the 0 → 2 curve (or wherever the map says a curve starts right after an intersection), so the gyro bridges it as in §31. To be checked on this drive's message stream with `c3_replay.py` first.
2. 0.4 m/s: the same swing would be larger at the higher speed, so (1) first is safer.
3. Past node 14 (roundabout 14 → 16, right turn 16 → 18, 18 → 12 → 0): needs recordings and offline checks before `cs_stop_s` is raised.

## 39. The 0 → 2 weak spot: white dashes and the intersection edge (2026-11-18, code only, not yet driven)

Chosen by the user: fix the weak spot from §38 first.

**What the camera saw at s ≈ 0.86** (frame `raw_frames/20261118_232957/1795044632.321_front.jpg`): no yellow line in view, so the edge search started in the middle of the image and took the first dark → bright step: **a white dash of the node 0 intersection markings**, not the road edge. One dash lay close enough to the expected curve that the fit passed (rms 0.105–0.108 cm), and two consecutive frames saw the same dash, so the confirmation let it through (−3.8 / −3.9 cm). The ordinary straight-line mode found no lines in those frames. The isolated −5.3 cm fit at s = 0.90 in §36 is the same spot.

**Change 1, `lane_camera.find_edge`: skip narrow white stripes** (`EDGE_MIN_RUN` = 40 px). A dash is a narrow bright stripe with road-dark (≤ `EDGE_DARK`) after it. The road edge is followed by the wide white surface, or the image border. Narrow stripes are skipped and the search carries on to the right. Everything else is unchanged: with `EDGE_MIN_RUN = 0` it gives exactly the old edge points (212/212 frames checked).
- It also skips the thin white parking-box outlines inside the road, which used to stop the search in some rows. On the parked hand-rolled and drive frames the edge now gets ~86 points per frame instead of 45–75, all on the real edge.
- **But on its own it isn't enough:** on the hand-rolled pass it let one edge fit through at s = 0.09 (node 0), +4.9 cm on a pass where the true offset is ≈ 0. That's the intersection's own edge, which bends away from the lane (§20). Dead reckoning then carried the +5 cm through the 0 → 2 curve (C2 offset mean +1.6 cm instead of −0.5).

**Change 2, `vehicle_control.py`: no edge-only fits from node 0 to ~1 m into the 0 → 2 curve** (`cs_edge_ignore = [(0.0, 1.05)]`, route distance s). Fits with the yellow line (Y, YE) are still used. The gyro bridges the rest, as over the blind stretch (§31: ±6 cm over 1.6 m). `cs_src` shows `zone` for those frames. Same in `c3_replay.py`.

**Replay of the third drive** (§38) with change 2:
- **Logged message stream** (old `lane_camera` fits): the s ≈ 0.86 fits aren't used. Dead reckoning stays at −2.2 … +0.8 cm over s 0–1.05. **At s = 1.53, where the edge comes back, it predicts +5.3 cm and the camera reads +5.8 cm.** In the drive the car really had moved ~5 cm left by then, because it steered at full lock on the dash fits. With the zone the estimate follows that motion correctly. So the dash fits were the error, and with the zone the controller wouldn't have made that steer.
- **Frames recomputed with change 1 as well:** same picture. Offset max 5.1 cm (that same s = 1.53 point), mean −0.7 cm, sd 1.3. 0 ignored by the gate, 2 held for confirmation.

**Limits:** the zone is specific to this route and this start (node 0). Other intersections on the rest of the route (node 14 roundabout, nodes 16, 18, 12) will need the same check before driving past node 14.

**Next:** the same drive again (copy `lane_camera.py`; `vehicle_control.py` uploads). Watch the 0 → 2 curve.

## 40. Fourth camera-steering drive (2026-11-18): planned stop reached; 0 → 2 now swung to the outer edge

Logs: `logs/car A/vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv`, `raw_frames/20261118_234715/` (replacing §38's logs). With the §39 changes.

- **Planned stop reached again** (s = 6.60 m, offset −1.2 cm). 2 → 4: −3.2 … −0.4 cm; node 4: −1.0 … −0.6; 4 → 14: −1.4 … −0.7.
- **The zone worked.** From node 0 to s = 0.37 the camera fits were clean (−2.1 … −1.0 cm, E and YE), and no dash fits were used.
- **User: in the 0 → 2 curve the car went closer to the outer edge.**

**Cause:**
- s 0.37–1.27: no usable fit; dead reckoning at −2.6 … +0.4 cm.
- **s 1.28–1.42: edge fits +9.5 → +2.1 cm (left), with headings −44 … −35°** (expected −16.5°). The edge points are on the real outer edge (frames `…673.338`, `…673.565`, `…673.779`), and the car is close to it. But at s ≈ 1.3 the camera's view (0.3–0.6 m ahead) takes in the curve end and the node 2 approach, where the edge widens away from the lane (§24), so offset and heading can't be separated. The jump gate ignored the first two, then the faster re-lock (§37, 3 consecutive within 1.5 cm) accepted them: the values changed smoothly by ~0.5 cm per frame.
- **The estimate jumped to +6 cm and the controller steered right (−10°), towards the outer edge.**
- One more fit, at s = 0.73, had diverged (−26 cm, +425°). The zone discarded it, but it shouldn't be sent at all.

**Confirmed by replaying this drive's message stream:** without those fits, dead reckoning follows the car's real motion (the right steer from the false re-lock): −1.3 → −4.8 → −6.0 cm. **At s = 2.11 the camera picks the edge up again and reads −6.3 cm (a match within 0.3 cm).** With the fits accepted, the estimate there was −2.5 cm.

**Fixes (code only, not yet driven):**
1. **Heading check, `vehicle_control.py`:** a fit whose heading is more than 15° from the expected heading is rejected entirely, offset included (`cs_max_head_fit`; `cs_src` shows `hbad`, `pose=` shows `hbad=`). Real fits in all drives so far were within 13°, including the real node 2 fits of §37 (11°).
2. **Diverged fits, `lane_camera.map_fit`:** |offset| > 30 cm or |heading| > 60° are dropped (`MF_MAX_D`, `MF_MAX_H`).
3. **`cs_dr_max` 1.8 → 2.3 m:** with (1) and the zone, the 0 → 2 curve is bridged by dead reckoning for up to 1.74 m (logged stream) or 1.98 m (5/s frames). Dead reckoning has matched the camera at the next good fit within 0.3 cm (here) and 0.7 cm (§39).

Replay with the fixes, logged stream: 342 fits, 1 held, 0 ignored by the gate. Offset mean −1.4 cm, sd 1.4, max 6.1 cm (that is the car's real right swing, which the estimate now tracks). No stop. Frames recomputed with the new `lane_camera.py`: max 5.5 cm, no stop.

**What this means for the 0 → 2 curve:** the front camera can't measure it reliably. The inner (yellow) line is out of view on this left curve, and the outer edge is corrupted by the intersections at both ends (node 0 dashes, node 2 widening). So the curve is driven on dead reckoning from the last good fit before it. That has been accurate so far (≤ 0.7 cm at re-acquisition), but it depends on a good offset/heading going in. The robust long-term fix is plan step 3b, **the left camera**, which sees the yellow line beside the car in this curve (§1, §21).

**Next:** the same drive again. If 0 → 2 stays clear of both edges, go on to 0.4 m/s. If not, the left camera for the curves.

## 41. Camera steering at 0.4 m/s (2026-11-18, setting only, not yet driven)

**Decided with the user:** keep the §40 version for now (the 0 → 2 curve on dead reckoning is acceptable for the moment) and go on to 0.4 m/s. The §40 fixes haven't been driven on their own; this drive tests them at 0.4 m/s.

**What doubles with the speed and what doesn't:**
- **Unchanged:** all acceptance rules and safety limits are in metres along the route (jump gate, confirmation, re-lock, edge zone, heading window, `cs_dr_max`, offset stop, planned stop). Dead reckoning works per metre too.
- **Doubles:** travel per camera frame (13/s: ~1.5 → ~3 cm), and travel during the camera delay.
- **GPS along-route position lags ~2 cm more** (readings arrive late). That's inside the ±5 cm the method needs (§31). `gps_front_ahead` (0.11 m) was measured at 0.2 m/s (§34).
- **Stanley:** the cross-track term `atan(k·e/v)` is half as strong at 0.4. GPS + table steering ran at 0.4 with `K_stanley` 1.5 without wobble (2026-10-28).

**Kinematic check** (`SteeringController` from the file, camera at 13/s with 60 or 120 ms delay, ±0.5 cm / ±1° noise, 0 → 2 bridged by ideal dead reckoning, start 2 cm off): the largest offsets were about the same at both speeds:

| | Approach | 0 → 2 | 2 → 4 | 4 → 14 |
|---|---|---|---|---|
| 0.2 m/s | 2.0 cm | 4.2–4.9 cm | 2.4–3.4 cm | ≤ 0.7 cm |
| 0.4 m/s | 2.0 cm | 4.4–4.6 cm | 3.5–3.7 cm | ≤ 0.5 cm |

Steering steps were smoother at 0.4. **The ~4–5 cm in the curves is the controller lagging at each curve entry**, even with perfect measurements: Stanley has no curvature feedforward (§14). That's a possible later improvement at either speed.

**Change:** `v_ref = 0.4` (0.2 kept as a comment). Same procedure as before (car_A_setup.md §3, "First camera-steering drive"), planned stop still at 6.6 m. If 0.4 is clearly worse, 0.3 is the in-between step. If it reaches the planned stop cleanly, the next steps are 0.4 m/s, and extending past node 14 (roundabout, right turn), checked offline on recordings first.

## 42. Planned stop removed (2026-11-19, code only, not yet driven)

**Decided by the user:** remove the hardcoded planned stop. `cs_stop_s` (6.6 m, just before node 14) and its stop check are gone from `vehicle_control.py`. The car now drives on past node 14 along `nodeSequence` `[0, 2, 4, 14, 16, 18, 12, 0]` until a safety stop (no camera fit for `cs_dr_max`, |offset| > `cs_max_offset`, camera silent for `cs_cam_timeout`) or a hand stop. `RoutePath` is cyclic, so nothing ends the drive at node 0 either.

Node positions along the route (from the removed comment): node 2 at 1.95 m, node 4 at 3.86, node 14 at 6.74, node 16 at 9.83, node 18 at 10.56, node 12 at 12.01, node 0 at 13.72.

**Untested past node 14:** the roundabout and the right turn haven't been checked offline (each intersection needs the same checks as node 0, §39). Stop by hand if needed.

## 43. Place-specific code removed: start zone, edge zone, GPS correction table (2026-11-19, code only, not yet driven)

**Decided by the user:** remove every hardcoded placement in `vehicle_control.py` that isn't a general solution (the route stays), for a holistic method instead of per-place fixes.

**Removed:**
1. **Start zone before node 0** (−0.9 < s < 0.15 m). The start position along the route now comes from `RoutePath.locate()`: the nearest route point to the first GPS reading **anywhere on the route**, among the points whose direction is within 90° of the GPS heading (`cs_start_head`), so a crossing road at an intersection isn't picked. The general check stays: refuse to start if the GPS is more than 15 cm from the route (`cs_start_lat`). `RoutePath` no longer extends a straight back from node 0 for s < 0; all s are taken modulo the route length L (13.75 m), so the straight before node 0 is now s ≈ 13.5 (the end of 12 → 0).
   - **Needed with it:** Stanley's waypoint index used to start at 0, which only worked because the car always started just before node 0. It is now set to the segment of the start position.
2. **`cs_edge_ignore`** (edge-only fits ignored at s 0–1.05 m, node 0 into the 0 → 2 curve, §39). The general rules from §39–§40 remain and act everywhere: `lane_camera.EDGE_MIN_RUN` (narrow white stripes aren't an edge), `cs_max_head_fit` (whole fit rejected if its heading is > 15° from the expected one), the jump gate and the confirmation rule. It also fixes lap 2: s isn't reset after a lap, so the zone would no longer have applied there.
3. **GPS correction table** (`gps_corrections`, `gps_correction_radius`, `use_gps_table`). `correct_gps()` applies only `gps_scale` (1.0). It was still active in camera-steering mode (`use_gps_table = True`): the start position, the along-route GPS nudges and their 15 cm route check used the corrected GPS. Camera steering never used it sideways. GPS-only steering (`use_camera_steer = False`) now runs on the uncorrected GPS.

   Old rows (x, y, dx, dy), for reference: node 0 (0.06, −0.06, +0.030, 0); mid 0 → 2 (0.27, −0.81, +0.021, +0.021); node 2 (1.26, −1.15, 0, +0.063); mid 2 → 4 (2.04, −0.88, −0.064, +0.064); node 4 (2.32, 0.00, −0.061, 0); mid 4 → 14 (2.29, 1.43, −0.036, 0); radius 0.35 m.

[qcar/c3_replay.py](../../qcar/c3_replay.py) updated the same way (start from `locate()` with the logged GPS heading, no edge zone).

**Replay of the §40 drive** (`20261118_234715`, logged message stream; its GPS lines were logged with the table on):
- Start s = 13.490 m = −0.26 m one lap on, the same as the car's `CAMSTEER: start at s = -0.26 m`.
- 363 fits (342 with the zone), 1 held, 0 ignored by the gate, 139 GPS corrections. Offset mean −1.4 cm, sd 1.5, max 6.6 cm (was 6.1); longest stretch without a fit 1.74 m. No stop.
- The 21 extra fits are edge-only fits at s 0.01–0.35 (node 0, start of 0 → 2): −1.4 … −2.1 cm, consistent with the YE fits beside them (−1.0 … −1.2). No dash fits (−4 … −5 cm at s ≈ 0.9, §38) in this stream: `EDGE_MIN_RUN` was already on in this drive.

**Not covered / to watch on the next drive:**
- A start away from the straight before node 0 is untested. On a curve or in an intersection the parked car may get no camera fit, so it starts with offset 0 and heading 0 and dead reckons.
- Without the table, the first GPS reading at the old start spot is ~3 cm different sideways (node 0 row +3 cm in x): still well inside the 15 cm start check and GPS route check.
- Whether node 0 dash fits come back now that the zone is gone (§38). If they do, the fix must be general, e.g. in `lane_camera.py`, not a zone.

## 44. Curvature feedforward tried in simulation: makes tracking worse, left off (2026-11-19, code only)

**Asked by the user:** add curvature feedforward to Stanley (the "later" item of §41).

**Code ([qcar/vehicle_control.py](../../qcar/vehicle_control.py)):** `SteeringController.update(p, th, speed, ff=0.0)` adds a feedforward angle. With `cs_ff = True`, camera steering steers `atan(cs_wheelbase · k)` for the map curvature k at `cs_ff_preview` m ahead, and measures Stanley's heading error from the expected heading on the arc (`hExp`) instead of the lane tangent, so the turn isn't counted twice. **`cs_ff = False`** (off) after the check below.

**Check: [qcar/c3_steer_sim.py](../../qcar/c3_steer_sim.py)** (new, PC only; replaces the unsaved §41 simulation). Kinematic bicycle on the full route, `SteeringController`/`RoutePath` from the file, camera 13/s with delay and noise (±0.5 cm, ±1°), ideal dead reckoning between frames, optional 100 ms steering-servo lag, start 2 cm off. Max |front-axle offset| in cm:

| Case | 0 → 2 | 2 → 4 | 14 → 16 | 16 → 18 | 18 → 12 |
|---|---|---|---|---|---|
| 0.4 m/s, 120 ms, servo 100 ms, **no FF** | 1.8 | 1.5 | 1.9 | 3.8 | 1.5 |
| same, FF, preview 0 | 2.3 | 2.6 | 2.3 | 5.6 | 3.5 |
| same, FF, preview 0.10 m | 4.5 | 5.3 | 6.3 | 12.3 | 6.9 |
| 0.2 m/s, 60 ms, no servo lag, **no FF** | 0.4 | 0.5 | 0.5 | 0.3 | 0.3 |
| same, FF, preview 0 | 1.7 | 1.8 | 2.2 | 4.8 | 2.5 |
| 0.4 m/s, perfect measurements, **no FF** | 0.06 | 0.14 | 0.07 | 0.07 | 0.09 |
| same, FF | 2.4 | 2.5 | 2.8 | 6.3 | 2.8 |

FF is worse in every case, and preview makes it worse still.

**Why:** Stanley here steers from the front axle. Its heading term points the front wheels along the lane tangent at the front axle, so (kinematically) the front axle follows the lane exactly, curve entries included. That is already a geometric feedforward. The added term assumes the steady-state relation on an arc (car heading = `hExp`, ~−16°); at a curve entry the car hasn't rotated yet, so the front wheels are turned in too early and cut inside.

**§41's "4–5 cm at curve entries even with perfect measurements" isn't reproduced** (this simulation: ≤ 0.15 cm perfect, ≤ 1.8 cm at 0.4 m/s with 120 ms delay and servo lag, except 3.8 cm on the tight 16 → 18 curve). The §41 script wasn't saved, so the difference can't be traced.

**What does put a wheel near the divider in left curves is geometry, not lag:** with the front axle on the lane centre, the rear axle runs on a smaller circle, √(R² − L²): on 0 → 2 (R = 0.885 m) that is **3.8 cm inside**. A general fix, if needed, would be to aim the front axle a little outside on arcs (about half of that), so the body is centred. Not done.

## 45. Feedforward refined: front axle only, ramped over L, exact tangent (2026-11-19, code only)

**Asked by the user:** (1) feedforward from the curvature at the front axle, no look-ahead (look-ahead is for speed planning, not steering); (2) build it up over one wheelbase L (ramp or low-pass) so it grows as the body rotates into the bend; (3) check that Stanley's heading error uses the local tangent at the front axle, not a point further down the curve.

**Code ([qcar/vehicle_control.py](../../qcar/vehicle_control.py)):**
- `cs_ff_preview` removed. New function `cs_feedforward()`, used by the car and by `c3_steer_sim.py`, with `cs_ff_mode`: `'instant'` (curvature at the front axle), `'ramp'` (curvature averaged over the last L of route), `'lowpass'` (first-order low-pass over distance, length L). `cs_ff = False`, `cs_ff_mode = 'ramp'`.
- (3) **Checked:** Stanley's heading error used the current waypoint segment's direction. Waypoints are 1 cm apart, so that was at most **0.59°** (~0.5 cm) ahead of the front-axle tangent. Now exact anyway: `SteeringController.update(..., tangent_ref)` and camera steering passes `route.heading(s)` at the front axle. **This is active with `cs_ff` off too** (a ≤ 0.59° change).

**Simulation** (`c3_steer_sim.py`, same model as §44), max |front-axle offset| in cm:

| Case | Mode | 0 → 2 | 2 → 4 | 14 → 16 | 16 → 18 | 18 → 12 |
|---|---|---|---|---|---|---|
| 0.4 m/s, perfect measurements | no FF | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| | instant | 2.4 | 2.4 | 2.8 | 6.3 | 2.8 |
| | **ramp** | 0.2 | 0.2 | 0.5 | 0.5 | 0.4 |
| | lowpass | 1.4 | 1.2 | 1.9 | 3.7 | 2.7 |
| 0.4 m/s, 120 ms, servo 100 ms | no FF | 1.7 | 1.4 | 1.8 | 3.8 | 1.5 |
| | instant | 2.3 | 2.7 | 2.4 | 5.5 | 3.5 |
| | **ramp** | 1.8 | 1.4 | 2.1 | 3.8 | 1.5 |
| | lowpass | 3.0 | 3.0 | 4.2 | 7.9 | 3.9 |
| 0.2 m/s, 60 ms, no servo lag | no FF | 0.4 | 0.5 | 0.5 | 0.3 | 0.3 |
| | **ramp** | 0.4 | 0.5 | 0.7 | 0.5 | 0.3 |

**Result:** the ramp over L fixes the over-steer at curve entries (instant), and is the best FF variant, but it is **the same as no FF** (within 0.1–0.5 cm, slightly worse). The low-pass lags at the far end and overshoots.

**Why the ramp equals no FF:** averaged over the last wheelbase, L·k̄ = heading(s) − heading(s − L) = −`hExp`. So `atan(L·k̄)` ≈ −`hExp`, which cancels the `hExp` taken out of the heading error: ψ + ff ≈ (lane tangent − car heading), plain front-axle Stanley. The small difference is atan(0.29) = 16.0° vs 0.29 rad = 16.4°. Front-axle Stanley is already the kinematically exact "feedforward ramped over L". A feedforward can only add something for delay (camera, servo), which needs look-ahead, and that is ruled out for steering.

**Decision for now:** `cs_ff` stays off; the exact front-axle tangent stays on. At 0.4 m/s with delays, the remaining few cm on the tight 16 → 18 curve come from delay, not missing feedforward.

## 46. Drive past node 14 (2026-11-19): roundabout left the lane; feedforward removed

Logs: `logs/car A/Fresh_Log/` (`vehicle_control.log`, `odometry_drive.csv`, `lane_camera.csv`, `lane_debug/`, `raw_frames` folder `20261119_234304/`, front + left). Code as after §45 (saved 17:20, drive 17:43): no planned stop, no start zone, no `cs_edge_ignore`, no GPS table, `cs_ff = False`, exact front-axle tangent; `v_ref` 0.4. Recomputing the steering from the logged offset/heading/speed matches "feedforward off" (median 0.13°, 95 % 0.78°; ramp 0.24°/1.60°, instant 0.32°/11.25°).

**Result:** ~10.5 m at 0.40 m/s, the furthest so far: straight before node 0 → 0 → 2 → 4 → 14 → roundabout → node 16. Stopped by `no camera fit for 2.30 m` at estimated s = 10.00 (start of 16 → 18). **User: in the roundabout the car did not stay in its lane; it stopped physically at node 16** (s = 9.83).

| Stretch (s mod 13.74) | Estimator |
|---|---|
| Start s = 13.53 (= −0.21) | whole-route `locate()`, +5.9 cm from the route by uncorrected GPS; YE fits −0.5 cm |
| Node 0 (0–0.22) | edge-only fits now used (zone removed): −2.0, −3.5 cm |
| 0 → 2 | dead reckoning −1.3 … −2.9 cm |
| Node 2 (2.10) | first fit after the gap −8.0 cm (E, rms 0.02–0.14), then smoothly −7.7 → −3.4 as the car steered back: most likely real, so ~5 cm DR drift through 0 → 2 |
| 2 → 4, 4 → 14 | E/YE fits −1.0 … −2.5 cm; GPS lost from s ≈ 4.6 (the known false solution, x ≈ 3.2) to the end |
| Roundabout 14 → 16 (7.6–9.83, R 0.68 m) | 63 of 70 frames no fit: front camera sees the wall/outer surface, the lane is out of view to the left (left camera sees the island and the lane, e.g. `…833.922_left.jpg`). Dead reckoning 2.3 m. Steering mean 25°, at the 30° limit for ~1 s (~22° needed to hold the arc) |

**Along-route position:** within ±3 cm of the GPS while the GPS was good (to s ≈ 4.4). At the stop the estimate was ~17 cm ahead (physical stop at node 16 vs s = 10.00; the one good GPS reading at the end said ~15 cm): ~3 % of the 5.6 m without GPS.

**Most likely cause of leaving the lane:** with no camera fit, the heading to the lane is gyro heading minus the map tangent at the *estimated* s. With s ahead by Δs on an arc, that tangent is off by κ·Δs = 1.48 × 0.10–0.17 m = **8.5–14°**, so the controller turns the car that much too far into the curve (inward, towards the island), 15–25 cm sideways per metre, while the dead-reckoned offset stays at −2 … −3 cm. The direction (inward) hasn't been confirmed by the user. If it went outward instead, the cause is the steering limit or slip. This mechanism doesn't explain the 0 → 2 drift (GPS good, Δs ±3 cm, drift outward).

**Feedforward removed (user's decision):** `cs_ff`, `cs_ff_mode`, `cs_feedforward()` and `SteeringController.update`'s `ff` argument are gone. The exact front-axle tangent (`tangent_ref`, §45) stays. `c3_steer_sim.py` now simulates plain Stanley only (same numbers as its earlier no-FF rows).

**Options (not done):** the left camera for left curves (step 3b); correcting s without GPS (curve starts/ends seen by the camera, gyro turn vs map); a dead-reckoning limit based on the expected heading error (κ × possible Δs) instead of a fixed 2.3 m; a curvature-based speed limit.

## 47. Steering trim −0.14 → −0.04, `K_soft`, curvature speed limit (2026-11-19, code only, not yet driven)

**Agreed with the user** after checking the steering limit on the §46 drive (and on the §40 drive of 2026-11-18). **All three can be reverted on their own; see "How to revert" below.**

### Finding: the steering trim was ~6° too far right

The achieved steering angle from the gyro and the speed, δ = atan(L · yaw rate / v) (gyro bias 0.005 rad/s from the parked second, 0.2 s smoothing, command taken 0.1 s earlier for the servo), against the controller's command (before the trim):

| Stretch / commands | Commanded | Achieved | Car |
|---|---|---|---|
| 4 → 14 straight, 11-19, 0.4 m/s | +6.0° | +0.3° | offset −2.0 cm, heading −1.6° |
| 4 → 14 straight, 11-18, 0.2 m/s | +5.9° | +0.2° | offset −1.0 cm, heading −1.5° |
| approach straight, 11-19 / 11-18 | +3.2° / +4.5° | −2.4° / −2.6° | |
| all commands 20–25° | 21.8° | 15.4° | |
| commands at the 30° limit | 29.4° | **21.2°** | |
| 0 → 2 arc (needs 16.8° = atan(L/√(R²−L²)), R 0.885) | 22.7° | 16.1° | |
| roundabout (needs **22.3°**, R 0.676) | 27.6° | **20.8°** | |

- On straights the controller had to command ~+6° to go straight. Stanley produces that only from an error, so the car sat 1–2 cm right of centre, pointing ~1.5° right: 1.6° + atan(1.5 · 0.02 / 0.4) = 5.9°. This is the "about −1 cm" of every camera drive (§36, §40).
- ~6° of the 30° left range was spent cancelling it. **In the roundabout the car could not reach the 22.3° the arc needs** (20.8° mean, 21.2° at the limit): ~0.09 /m too wide, ~20 cm outward over 2.2 m. **This revises §46:** the car most likely left the roundabout lane **outward**, and the ~15 cm along-route error only made the controller ask for more left steering, which it couldn't get. On 0 → 2 the car was slightly wide (16.1° vs 16.8°), consistent with the −8 cm (right/outward) at node 2.
- The Quanser library clips `delta + steering_trim` at ±0.6 rad (`pal/products/qcar.py` `write()`), so the controller's ±30° (`maxSteeringAngle`) is the limit that applies.
- Straight-ahead point: trim + 5.7° ≈ −8.0° + 5.7° = −2.3° ≈ **−0.04 rad**. `−0.14` was tuned on 2026-10-28 from the GPS cross-track average, which had its own GPS biases.

### Changes ([qcar/vehicle_control.py](../../qcar/vehicle_control.py))

1. **`steering_trim = -0.04`** (was −0.14, kept as a comment). Expected: straights centred (offset ≈ 0, heading ≈ 0), ~27° of left steering reachable instead of ~21° (the roundabout needs 22.3° steady, up to ~26° at its entry in the simulation). Right-turn headroom drops by the same ~6°: 16 → 18 needs 24.4° steady, ~23–25° in the simulation; ~27° expected available. **Check on the next drive:** `delta` on straights ≈ 0 and `cs_d_cm` ≈ 0; the steering never sitting at ±30° for long.
2. **`K_soft = 0.05`** m/s: Stanley's cross-track term is atan(`K_stanley` · e / (v + `K_soft`)), in `SteeringController` (new `k_soft` argument, both controllers). Keeps it finite near standstill (at the start it jumped to ±90° and was clipped). At 0.4 m/s the cross-track gain is 11 % weaker, at 0.2 m/s 20 %. Simulation: ≤ 0.1 cm change.
3. **Curvature speed limit** (`use_curve_speed = True`, camera steering only, since it needs the position along the route): speed target = min(`v_ref`, √(`v_curve_alat` / k)) for the largest |map curvature| k from `cs_wheelbase` behind to `v_curve_ahead` = 0.5 m ahead of the front axle (look-ahead used for speed planning only, not steering), never below `v_curve_min` = 0.2 m/s. `v_curve_alat` = 0.13 m/s²: straights 0.40, 0 → 2 / 2 → 4 0.34, roundabout 0.30, 16 → 18 0.28 m/s. The speed PI controller follows `vTarget` instead of `v_ref`. `pose=` lines show `v_target=`.

### Simulation ([qcar/c3_steer_sim.py](../../qcar/c3_steer_sim.py), now with `K_soft`, the speed limit and a first-order speed loop, 0.3 s)

Max |offset| (cm), and the largest |steering| (deg) per stretch:

| Case | 0 → 2 | 2 → 4 | 14 → 16 | 16 → 18 | Lap |
|---|---|---|---|---|---|
| 120 ms, servo 100 ms, before (0.4 const, `K_soft` 0) | 1.7 | 1.4 | 1.8 | 3.8 | 35.1 s |
| `K_soft` only | 1.8 | 1.3 | 1.9 | 3.9 | 35.1 s |
| `K_soft` + speed limit (now) | 1.3 | 1.4 | 1.3 | **2.0** | 40.9 s |
| steering, now (deg) | 17.8 | 19.2 | 24.6 | 22.9 | |
| 60 ms, no servo lag, before / now | 0.7 / 0.8 | 0.5 / 0.5 | 0.7 / 0.4 | 0.8 / 0.5 | 35.0 / 40.8 s |

The simulation assumes the steering reaches what is commanded (perfect trim), so it checks the controller and the speed plan, not the trim itself.

**Afterwards (user, 2026-11-19):** `use_curve_speed = False` for the next drive, so the trim is tested on its own first (drive A), then the speed limit (drive B). See "Where to pick up" at the top.

### How to revert (each on its own)

| Change | Revert |
|---|---|
| Trim | `steering_trim = -0.14` (the commented line above it) |
| `K_soft` | `K_soft = 0.0`: exactly the old controller |
| Speed limit | `use_curve_speed = False`: constant `v_ref` again |

## Files

| File | What |
|---|---|
| [qcar/csi_snapshot.py](../../qcar/csi_snapshot.py) | One frame + yellow mask from each CSI camera (§1) |
| [qcar/lane_camera.py](../../qcar/lane_camera.py) | Lane offset from the front camera (§3); UDP sender (§10); `--save-raw` (§16) |
| [qcar/curve_offline_test.py](../../qcar/curve_offline_test.py) | PC-only offline test of boundary + known curvature on raw frames (§19) |
| [qcar/curve_start_test.py](../../qcar/curve_start_test.py) | PC-only C1 test: change points seen by the camera, yellow and edge separately (§23) |
| [qcar/odometry_log.py](../../qcar/odometry_log.py) | Encoder + gyro logger for hand-rolled passes, `odometry.csv` (§24, §25) |
| [qcar/c3_steer_sim.py](../../qcar/c3_steer_sim.py) | PC-only kinematic simulation of camera steering (§44–§47; `K_soft` and curvature speed limit since §47) |
| [qcar/c2_offline_test.py](../../qcar/c2_offline_test.py) | PC-only C2 emulation: map shape at the encoder position + camera fits + dead reckoning (§25) |
| `logs/car A/odometry.csv`, `raw_frames/20261117_204844/`, `raw_frames/c2_*` | C2 hand-rolled pass and results (§25) |
| `logs/car A/raw_frames/20261117_191853/`, `20261117_200303/` | Hand-rolled 0 → 2 → 4 frames (§18) and drive 1 frames (§21), copied back from the car; C1 results in `curve_start_*.csv` (§24). `curve_test_20261117_191853/` (§19, §20 plots) is empty; regenerate with `curve_offline_test.py` if needed |
| `logs/car A/raw_frames/20261117_201140/`, `20261117_201313/`, `curve_start_*.csv` | Two drives that crossed left on 0 → 2 (§22, §23) and the C1 results |
| [qcar/calib_capture.py](../../qcar/calib_capture.py), [qcar/calib_solve.py](../../qcar/calib_solve.py) | Checkerboard frames on the car; calibration on the PC (§27, §28) |
| [qcar/road_check.py](../../qcar/road_check.py) | PC-only check of the calibration against strips, lane width and curves (§29) |
| `logs/car A/calib_frames/` | Checkerboard frames and `calib_result_*` (§27) |
| [qcar/vehicle_control.py](../../qcar/vehicle_control.py) | `t_epoch=` in the `pose=` line (§3); camera lane correction, `use_gps_table`, `v_ref = 0.2` (§10) |
| [docs/plans/camera_lane_correction.md](../plans/camera_lane_correction.md) | The plan: methods, steps, short step results |
| [docs/car_A_setup.md](../car_A_setup.md) | Current settings, run steps (start `lane_camera.py` first), log format |
| `logs/car A/snapshots/` | Step 0 images (§1) |
| `logs/car A/vehicle_control_lapA_no_camera.log`, `vehicle_control_lapB_camera.log` | Laps A and B (§5) |
| `logs/car A/check_pose.log` | Unusable survey attempt (§7) |
| `logs/car A/lane_camera_left_d_2.5cm.csv`, `lane_camera_right_d_2.5cm.csv` | Ruler check (§9) |
| `logs/car A/vehicle_control.log`, `lane_camera.csv`, `lane_debug/` | First step 5 drive (§11). Earlier drives' logs were overwritten |
