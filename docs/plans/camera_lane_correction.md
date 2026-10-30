# Plan: Camera Lane Correction

**Status:** methods chosen (2026-10-30), not started. From the [new-route record](../history/2026-10-28_new_route_investigation.md) §15.

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

- Start with the **side camera facing the yellow divider** (left camera in the right lane, right camera in the left lane) **plus the front camera** for the curve ahead. The rear camera isn't needed for lane offset.
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

0. **Look first (next step):** on car A, with the car parked in the right lane and then the left lane, save a frame from each CSI camera. Check which way each one faces, how much floor it sees, and whether the yellow divider (and white edge line) is clearly in view. This confirms which cameras to use before calibrating.
1. **Calibrate:** fisheye intrinsics and floor mounting for the chosen CSI cameras on car A. Check by placing tape at known offsets and comparing with the measured offsets.
2. **Measure only:** run the CSI lane process during a normal drive and log the offset, heading error, confidence and (camera − GPS). The car's behaviour doesn't change.
3. **Check it:** compare the logged offsets with what the car visibly does, especially on the 0 → 2 and 2 → 4 curves, and compare (camera − GPS) with the existing correction table.
4. **Use it in the steering:** blend + learned bias, with the watchdog. Then decide whether the correction table is still needed.
5. **Intersections:** after the lane behaviour holds up.
