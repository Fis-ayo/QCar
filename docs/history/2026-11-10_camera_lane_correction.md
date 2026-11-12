# Camera Lane Correction: Report

> **Combined report for the [camera lane correction plan](../plans/camera_lane_correction.md)**, car A (`192.168.0.151`), from 2026-11-10. New work on this plan is added here as a new dated part, so all results stay in one doc. Current settings: [car_A_setup.md](../car_A_setup.md).

**Goal:** see the left and right lane boundaries with a camera, so the car can stay centred in its lane without depending on the lidar GPS and its correction table. Since 2026-11-11 the goal is for the camera to **replace** the `gps_corrections` table completely, not blend with it.

**Status so far:**
- **Front camera calibrated** (no checkerboard). [qcar/lane_camera.py](../../qcar/lane_camera.py) measures the car's offset from the lane centre on straights, accurate to within 1 cm (ruler check, §9). Tight curves can't be measured yet.
- **Camera steering written** (plan step 5, §10). In the first drive (§11) the car started straight, but it crossed the divider on the 0 → 2 curve, where the front camera is blind.
- **Open:** a camera method for curves (§12; assessment of a proposed pipeline in §14) and the localization failure on 4 → 14 after the recalibration (§6).
- **Where to pick up: §15.**

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

## Files

| File | What |
|---|---|
| [qcar/csi_snapshot.py](../../qcar/csi_snapshot.py) | One frame + yellow mask from each CSI camera (§1) |
| [qcar/lane_camera.py](../../qcar/lane_camera.py) | Lane offset from the front camera (§3); UDP sender (§10) |
| [qcar/vehicle_control.py](../../qcar/vehicle_control.py) | `t_epoch=` in the `pose=` line (§3); camera lane correction, `use_gps_table`, `v_ref = 0.2` (§10) |
| [docs/plans/camera_lane_correction.md](../plans/camera_lane_correction.md) | The plan: methods, steps, short step results |
| [docs/car_A_setup.md](../car_A_setup.md) | Current settings, run steps (start `lane_camera.py` first), log format |
| `logs/car A/snapshots/` | Step 0 images (§1) |
| `logs/car A/vehicle_control_lapA_no_camera.log`, `vehicle_control_lapB_camera.log` | Laps A and B (§5) |
| `logs/car A/check_pose.log` | Unusable survey attempt (§7) |
| `logs/car A/lane_camera_left_d_2.5cm.csv`, `lane_camera_right_d_2.5cm.csv` | Ruler check (§9) |
| `logs/car A/vehicle_control.log`, `lane_camera.csv`, `lane_debug/` | First step 5 drive (§11). Earlier drives' logs were overwritten |
