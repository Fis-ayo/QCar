# Plan: Camera Lane Correction

**Status (end of 2026-11-11):** steps 0–2 and 4 done; car A recalibrated; **step 5 written and driven once: straight start, but it crossed the divider on the 0 → 2 curve.** **Open: a camera method for curves** (the user is researching one; option discussed: the visible edge + the map's curvature, [report](../history/2026-11-10_camera_lane_correction.md) §12). Where to pick up: that report, §13. The goal is now for the camera to **replace** the `gps_corrections` table, not blend with it. All results so far: one combined [report](../history/2026-11-10_camera_lane_correction.md). Known limits: the localization fails on 4 → 14 at y ≈ 1.0–1.3 after the recalibration (every drive stops there, not yet investigated), and the front camera sees nothing from node 0 through the 0 → 2 curve. Methods chosen 2026-10-30, from the [new-route record](../history/2026-10-28_new_route_investigation.md) §15.

## Why

The GPS correction table ([car_A_setup.md](../car_A_setup.md) §1) treats the symptom: it adds back an error measured once in the lidar readings. It is only as good as those measurements and is invalid after any room or calibration change. Walls near the lower part of the map would make the lidar position more accurate, but have the same dependence on the room. Even at its best, the lidar position (±2–3 cm) uses most of the ~4 cm clearance.

**Camera lane correction measures the thing that matters:** the car's sideways position between the painted lines, independent of the lidar localization.

| | Correction table (now) | Walls + recalibrate | Camera correction |
|---|---|---|---|
| What it measures | Nothing new: adds back an error measured once | Better lidar position | The car's actual place in the lane |
| Survives room or calibration changes | No | No | Yes |
| Works on car B without retuning | No | No | Mostly (same camera mounting) |
| Effort | Done | Moderate | Most |

## What it does and doesn't replace

**It does not replace the GPS.** The lidar position is still needed to know where the car is on the map, when to turn and which route to follow. The camera only corrects the sideways position within the lane.

**Keep the correction table until the camera is proven.** At intersections, or wherever the lines aren't visible, the car falls back to GPS steering, and the table keeps that fallback accurate on the lower part.

## Chosen methods

### 1. Cameras: CSI, not RealSense

Lane keeping is a 2D floor problem. The CSI fisheye cameras look down at the line next to the car, which is the sideways position we want to measure. The RealSense stays with YOLO for object detection.

- **Front camera only** (decided 2026-11-10). It sees both boundaries: the yellow line on the left (always on the car's left on this route, see step 0) and the right road edge, plus the curve ahead. One camera means one calibration and less CPU.
- Its limits: it measures ahead, so the offset at the front axle is extrapolated from the fitted line. And when the view ahead is blocked (e.g. by car B in front), there's no measurement and the car falls back to GPS + bias.
- **Left camera: optional later addition**, as a second source of yellow-line points beside the car. Add it if the step 2 logs show the front camera struggling on curves or behind another car. The right and rear cameras aren't needed.
- Nothing in `qcar/` reads the CSI cameras yet, so this is a **new process**, separate from [qcar/yolo_server.py](../../qcar/yolo_server.py). It doesn't compete with the RealSense.
- The calibration belongs to the car, not the room: fisheye intrinsics from a checkerboard ([skills_activities/02-image_interpretation/](../../skills_activities/02-image_interpretation/) already does this at 820×410), and the camera mounting from tape at known positions on the floor.

### 2. Finding the line: colour + edges

- **Colour:** HSV threshold for yellow (and white for edge lines). Normalise V first (e.g. CLAHE on V, or a luminance-invariant mask) so glare and room lighting don't move the threshold.
- **Edges:** Sobel or Canny. They catch a line whose colour has been washed out by glare.
- **A pixel counts only if it passes both tests.** Edges alone give false lines from crosswalks, stop lines, curb edges and other cars.
- **Region of interest:** only search where the line can realistically be: the side of the car where the yellow divider is expected (known from the route), ± a margin.
- **Open choice, decide on real frames:** run the detector on the raw image (cropped) or on the top-down view. The top-down view is attractive because the lines are straight and a constant width there, and the warp is a precomputed remap, so it's cheap.

### 3. Pixels to cm: per-camera IPM into car coordinates

- Each camera gets its own undistort + inverse perspective mapping (IPM) onto the floor, expressed in **the car's coordinate frame** (`LaneKeeping` in `hal.content.qcar_functions` has the IPM pieces).
- **Combine the measurements, not the images:** the line points from each camera are projected into the car frame and fitted together. No stitched surround-view image or overlap blending is needed.
- The fit (line, or low-order polynomial for curves) gives two numbers: **sideways offset at the front axle** and **heading error** relative to the lane. Both feed the Stanley controller.
- Offset = distance to the yellow line − half the lane width, on the side given by the route. A visible white edge line is a second reference and a check on lane width.

### 4. Combining with GPS: blend + learned bias, switch as watchdog

A full Kalman filter is not worth the development time: the lidar localization already does map matching, and learning a bias with a low-pass filter is effectively a one-state Kalman filter on that bias.

- **Main: confidence-weighted blend.** Cross-track error = blend of the camera offset and (GPS + learned bias), weighted by detection confidence (inlier count, fit quality, lane-width check). It reacts quickly when the GPS error changes along the route.
- **Main: learned GPS bias.** While the camera is confident, low-pass filter (camera − GPS) into a running bias. When the lines disappear, the car steers on GPS + that bias, a self-updating replacement for the correction table.
- **Watchdog: switch.** If the camera and GPS disagree by more than a limit (e.g. half a lane), or the detection is lost, ignore the camera (weight 0) and freeze the bias. Steering then works as it does now.

### 5. Intersections: later

The detection is the same, but the behaviour isn't: crossing markings confuse the detector. **Lanes first.** Initially, inside intersection zones (known from the GPS route) the camera weight is 0 and the bias is held. Later options:

- Make sure the car is centred and straight before entry, so the GPS-only turn starts well.
- Hold the learned bias through the intersection.
- Pick up the exit lane's line as early as possible (the fisheye side cameras see it sooner).

## Plan (measure first, then steer)

0. **Look first (done):** on car A, with the car parked in the right lane and then the left lane, save a frame from each CSI camera. Check which way each one faces, how much floor it sees, and whether the yellow divider (and white edge line) is clearly in view. This confirms which cameras to use before calibrating.
   **Done 2026-11-10** ([qcar/csi_snapshot.py](../../qcar/csi_snapshot.py), images in `logs/car A/snapshots/`, 820×410, right/left lane × straight/curve):
   - The yellow divider is on the car's **left** in all four positions, including the left lane (driven in the opposite direction). So the side is fixed: left = yellow.
   - **The right boundary is not a painted line.** It is the edge between the dark road and the white surface next to it.
   - **Left camera:** sees the yellow line as an arc at the bottom of the image, only at the left and right corners. The part of the line directly beside the camera is below the frame, and the car body hides the extreme corners. The line can still be measured after undistort + IPM.
   - **Front camera:** sees the yellow line ahead clearly, and also the road/white edge on the right. It is the only camera that sees both boundaries.
   - **Right camera:** sees no road at all, only the white surface. With the car parked by hand in the centre, the right road edge is in its blind spot.
   - **Back camera:** not needed. It picks up intersection markings.
   - **Example yellow threshold** `[10,50,100]–[45,255,255]`: finds the divider in every left and front image despite the pink colour cast. False positives (brown boxes, far objects) appear only above the horizon, so a floor region of interest removes them.
   - **Camera chosen:** front only, with the left camera as an optional later addition (see §1).
1. **Calibrate (front camera, done 2026-11-10), no checkerboard or tape needed:**
   - **Fisheye:** reuse the CSI intrinsics and distortion at 820×410 from [image_interpretation.py:290](../../skills_activities/02-image_interpretation/image_interpretation.py#L290). Tested 2026-11-10 on the front snapshots: poles, floor edge and yellow line come out straight. Do a checkerboard calibration only if later checks show a distortion error.
   - **Mounting:** measure the camera height and its distance ahead of the front axle with a ruler. Estimate the pitch from the horizon in the undistorted image.
   - **Check with the road itself:** with the car parked centred, the yellow line and the right edge should come out at equal distances, and their spacing should match the lane width. Tape is needed only if these checks disagree.

   **First results (2026-11-10, front camera, car A, from the two straight snapshots):**
   - Measured with a ruler: camera 11 cm above the floor (to the top of the camera, so the lens centre is ~1 cm lower), 10 cm ahead of the front axle (assumed ahead). Lane width 25.7 cm, from the centre of the yellow line to the road edge.
   - Height to the top of the camera explains only ~10 % of the scale error below. The rest is most likely the borrowed intrinsics. No need to re-measure: the lane-width scaling and the fixed factor absorb both.
   - Pitch from the vanishing point of the yellow line and the right edge: −2.66° and −2.68° (the two snapshots agree).
   - Both lines are seen from about 35 cm to 1 m ahead (on the uncorrected scale). Line fit scatter: 0.3 cm (yellow), 0.1 cm (edge).
   - **Centring works:** with the car parked centred by hand, the lane centre comes out −0.2 cm and −0.4 cm from the car's centre, and the heading relative to the lane within 0.4°. This doesn't depend on the scale, because an error in scale stretches both distances equally.
   - **Scale is off:** the projected lane width comes out 35.5 cm instead of 25.7 cm (×1.38, the same in both snapshots). Likely causes: the borrowed intrinsics (they come from another calibration) or the camera height. A ×1.38 error would need the lens at 8 cm instead of 11 cm.
   - **Proposed fix:** where both lines are visible, scale by the known lane width: offset = (yellow + edge) / (yellow − edge) × 25.7 cm / 2. Where only the yellow line is visible, use a fixed scale of 25.7 / 35.5 = 0.724. Confirm with the step 4 ruler check. Calibrate with a checkerboard only if that check fails.
2. **Measure only (next step):** run the CSI lane process during a normal drive and log the offset, heading error, confidence and (camera − GPS). The car's behaviour doesn't change.

   **Written 2026-11-10:** [qcar/lane_camera.py](../../qcar/lane_camera.py) (front camera, logs `lane_camera.csv` + overlay images in `lane_debug/`). Tested offline on the snapshots:
   - **Straights:** offset +0.2 / +0.3 cm, lane width 25.3 cm (true 25.7), heading < 0.5°, confidence 1.0.
   - **Tight curves can't be measured with the front camera alone.** The yellow line is visible only from ~28 cm ahead, over a short stretch, and extending a curving line back to the axle is unreliable: line, quadratic and circle fits all gave wrong offsets. These frames are logged as `mode=curve` with confidence 0 (heading > 10° or curvature > 0.5 /m).
   - **Options for curves, later:** the left camera (step 3b) sees the line beside the car. Or fit the line with the curvature known from the map route, leaving only offset and heading free.
   - `vehicle_control.py` now ends each `pose=` line with `t_epoch=` (2026-11-10), the same clock as `lane_camera.csv`, so camera − GPS can be matched by time.

   **First drive with the camera (2026-11-10)**, logs in `logs/car A/` (`vehicle_control.log`, `lane_camera.csv`, `lane_debug/`):
   - **The run stopped after ~10 s on the 0 → 2 curve:** GPS readings 0.5–3 m off from 1.5 s in, then the 2 s safety stop. The last good run at 0.4 had 0 rejections.
   - **The camera script was using too much CPU:** ~70 frames/s processed although the camera delivers ~30. 62 % were repeats of the previous frame, so one core was busy most of the time. That could have starved the localization. A room change would produce the same GPS jumps, so this is not proven.
   - **Fix:** process only new frames, at most 15/s (`--max-rate`).
   - **Camera:** parked at the start, both lines, offset ≈ 0, width 25.5–25.8 cm, stable for 24 s. **On the 0 → 2 curve the front camera loses the yellow line**: it's out of view on the left (`none`/`curve` frames). This supports step 3b (left camera).
   - **Next:** a lap without `lane_camera.py` (is the GPS fine on its own?), then a lap with the fixed script.

   **Laps A and B (2026-11-10)**, `logs/car A/vehicle_control_lapA_no_camera.log`, `vehicle_control_lapB_camera.log`, `lane_camera.csv` (lap B):
   - **The GPS problem isn't caused by the camera.** Lap A (no camera) completed a full lap (37 s) but with 29 rejections. Lap B (camera) had rejections in the same places, then lost the localization near (2.25, 3.5) and stopped after 20 s. Lap A had one rejection at that same spot with the same wrong solution (≈ (0.4, 4.54, 180°)). Most rejections are on the lower part (0 → 2 → 4), jumping 0.5–2.5 m towards +y. On 2026-10-28 the same route had 0 rejections, so the localization has got worse since then (room change? recalibration needed?). A small extra effect from the camera load can't be ruled out from one lap each.
   - **CPU fix works:** 13.8 frames/s processed, median 10 ms per frame.
   - **Camera vs GPS, 4 → 14 straight (x ≈ 2.28, y 0.9–2.7, both lines, 9 matched samples):** camera offset +0.7 cm (sd 1.3), GPS cross-track −3.2 cm (sd 1.0, sign flipped to left = +). **Camera − GPS = +3.9 cm (sd 1.5):** the camera says centred, the GPS says ~3 cm right. The overlay image there looks centred. This is the kind of steady bias step 5 would learn.
   - **Where the front camera sees nothing useful:** the 0 → 2 curve and intersection (`curve`/`none`); around node 4 and the start of 4 → 14 (x ≈ 2.25, y −0.1 to 0.9), where only the right edge is seen (`edge` mode, yellow out of view).
   - `lane_camera.py` now empties `lane_debug/` at start (lap B's folder still held lap 1's images).
3. **Check it:** compare the logged offsets with what the car visibly does, especially on the 0 → 2 and 2 → 4 curves, and compare (camera − GPS) with the existing correction table.
3b. **Add the left camera:** same detector and IPM, with the yellow-line points beside the car added to the front camera's points in car coordinates. Check its offsets against the front camera's on the logged drive. This is a backup when the view ahead is blocked (car B in front) and gives a better offset on curves.
4. **Before steering, a ruler check:** park the car at a few known offsets (ruler from the car's side to the yellow line) and compare with the camera offset. Errors in step 2 are harmless because the camera doesn't steer yet, but here the cm values must be right.
   **Done 2026-11-11** ([report](../history/2026-11-10_camera_lane_correction.md) §9): 3 spots, all within 1 cm (`both` mode), but only ±0.6 cm could be tested. Accepted for step 5; **if the camera steering misbehaves, come back and do the tighter test** in that report (measure both clearances at the front wheels, offsets up to ±6 cm).
5. **Use it in the steering:** blend + learned bias, with the watchdog. Then decide whether the correction table is still needed.
   **Written 2026-11-11, not yet driven** (details: [car_A_setup.md](../car_A_setup.md) §1, "Camera lane correction"). Changed from the original idea: **the table is off** (`use_gps_table = False`), and **instead of a confidence blend, the camera's learned bias is applied directly** (the steering point is shifted sideways by `cam_bias`). While the camera is trusted, that makes the cross-track equal the camera offset. Otherwise the bias is held. The bias is learned while parked during `startDelay`. This targets the right swing at the start of the 11-10/11-11 drives: the GPS said the car was ~8 cm left while the camera saw it centred. On 0 → 2 → 4 the front camera is blind, so the bias learned at node 0 is held through the curves. **First test:** start at node 0 at `v_ref = 0.2`, watch whether the start is straight and how the curves behave with the held bias. Fallbacks if the curves drift: the old table on the curves only, or step 3b.
   **First drive (2026-11-11, [report](../history/2026-11-10_camera_lane_correction.md) §11):** the start is straight now (bias −5.7 cm learned while parked), but the car crossed the divider at the 0 → 2 exit (camera +7.6 cm) with the node 0 bias held through the blind curve. **Curves need their own camera measurement** before step 5 can work on this route.
6. **Intersections:** after the lane behaviour holds up.
